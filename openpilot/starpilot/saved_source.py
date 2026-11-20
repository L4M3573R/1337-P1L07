"""Bounded saved-setting reads for the shared feature owner."""

import os
import stat

# (path, limit) -> (file signature, bytes) of the last complete read; None = off (every process except the UI).
_unchanged: dict[tuple[str, int], tuple[tuple[int, int, int, int, int], bytes]] | None = None


def keep_unchanged_reads(enabled: bool = True) -> None:
  """Opt a process in to reusing the bytes of a saved file that has not changed since its last complete read.

  The UI rereads the same small settings files every frame or every second. With this on, a read first stats the
  file; when device, inode, size, mtime and ctime (ns) all match the last complete read, those bytes are returned
  without opening the file. Any write, replace or removal changes the signature and gets the full verified read.
  """
  global _unchanged
  _unchanged = {} if enabled else None


def _signature(st: os.stat_result) -> tuple[int, int, int, int, int]:
  return st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns


def _read_verified(path, limit: int) -> tuple[bytes | None, bool, tuple[int, int, int, int, int] | None]:
  flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0)
  try:
    fd = os.open(path, flags)
  except FileNotFoundError:
    return None, True, None
  except OSError:
    return b"", False, None
  try:
    before = os.fstat(fd)
    if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
      return b"", False, None
    with os.fdopen(fd, "rb", closefd=False) as source:
      raw = source.read(limit + 1)
    after = os.fstat(fd)
    current = os.stat(path, follow_symlinks=False)
    if (len(raw) > limit or not stat.S_ISREG(after.st_mode) or
        (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) !=
        (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) or
        (current.st_dev, current.st_ino) != (after.st_dev, after.st_ino)):
      return b"", False, None
    return raw, True, _signature(after)
  except OSError:
    return b"", False, None
  finally:
    os.close(fd)


def read_saved(params, key: str, limit: int) -> tuple[bytes | None, bool]:
  """Return complete bytes/absence and readability; never authorize a prefix."""
  path = params.get_param_path(key)
  cache = _unchanged
  if cache is None:
    raw, readable, _ = _read_verified(path, limit)
    return raw, readable
  try:
    st = os.stat(path, follow_symlinks=False)
  except FileNotFoundError:
    cache.pop((path, limit), None)
    return None, True
  except OSError:
    st = None
  if st is not None and stat.S_ISREG(st.st_mode):
    hit = cache.get((path, limit))
    if hit is not None and hit[0] == _signature(st):
      return hit[1], True
  raw, readable, signature = _read_verified(path, limit)
  if readable and raw is not None and signature is not None:
    cache[(path, limit)] = (signature, raw)
  else:
    cache.pop((path, limit), None)
  return raw, readable
