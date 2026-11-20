import os

import pytest

from openpilot.starpilot import saved_source
from openpilot.starpilot.saved_source import keep_unchanged_reads, read_saved


class FakeParams:
  def __init__(self, root):
    self.root = root

  def get_param_path(self, key):
    return os.path.join(self.root, key)


@pytest.fixture
def params(tmp_path, monkeypatch):
  # Files written by a test are seconds old at most; let them count as settled unless a test says otherwise.
  monkeypatch.setattr(saved_source, 'RECENT_CHANGE_NS', -10**18)
  yield FakeParams(str(tmp_path))
  keep_unchanged_reads(False)


def write(params, key, data):
  with open(params.get_param_path(key), 'wb') as f:
    f.write(data)


def test_off_by_default_reads_every_time(params, monkeypatch):
  assert saved_source._unchanged is None
  write(params, 'K', b'abc')
  assert read_saved(params, 'K', 16) == (b'abc', True)
  monkeypatch.setattr(saved_source.os, 'open', lambda *a, **k: (_ for _ in ()).throw(PermissionError()))
  assert read_saved(params, 'K', 16) == (b'', False)


def test_unchanged_file_is_not_reopened(params, monkeypatch):
  keep_unchanged_reads()
  write(params, 'K', b'abc')
  assert read_saved(params, 'K', 16) == (b'abc', True)
  monkeypatch.setattr(saved_source.os, 'open', lambda *a, **k: (_ for _ in ()).throw(AssertionError('reopened')))
  assert read_saved(params, 'K', 16) == (b'abc', True)


def test_in_place_rewrite_with_same_size_and_mtime_is_reread(params):
  keep_unchanged_reads()
  write(params, 'K', b'abc')
  path = params.get_param_path('K')
  before = os.stat(path)
  assert read_saved(params, 'K', 16) == (b'abc', True)
  write(params, 'K', b'xyz')
  os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))  # only ctime still moves
  assert read_saved(params, 'K', 16) == (b'xyz', True)


def test_replace_remove_and_recreate(params):
  keep_unchanged_reads()
  write(params, 'K', b'one')
  assert read_saved(params, 'K', 16) == (b'one', True)
  tmp = params.get_param_path('K.tmp')
  with open(tmp, 'wb') as f:
    f.write(b'two')
  os.replace(tmp, params.get_param_path('K'))
  assert read_saved(params, 'K', 16) == (b'two', True)
  os.remove(params.get_param_path('K'))
  assert read_saved(params, 'K', 16) == (None, True)
  write(params, 'K', b'six')
  assert read_saved(params, 'K', 16) == (b'six', True)


def test_unreadable_results_are_never_cached(params):
  keep_unchanged_reads()
  write(params, 'BIG', b'0123456789')
  assert read_saved(params, 'BIG', 4) == (b'', False)
  assert read_saved(params, 'BIG', 4) == (b'', False)
  assert ('%s' % params.get_param_path('BIG'), 4) not in saved_source._unchanged
  write(params, 'T', b'ok')
  os.symlink(params.get_param_path('T'), params.get_param_path('L'))
  assert read_saved(params, 'L', 16) == (b'', False)
  assert read_saved(params, 'L', 16) == (b'', False)


def test_limit_is_part_of_the_cache_key(params):
  keep_unchanged_reads()
  write(params, 'K', b'abcdef')
  assert read_saved(params, 'K', 16) == (b'abcdef', True)
  assert read_saved(params, 'K', 3) == (b'', False)
  assert read_saved(params, 'K', 16) == (b'abcdef', True)


def test_recently_changed_file_is_always_reread(params, monkeypatch):
  # A write in the same coarse clock tick as the last read would leave the file times unchanged.
  monkeypatch.setattr(saved_source, 'RECENT_CHANGE_NS', 1_000_000_000)
  keep_unchanged_reads()
  write(params, 'K', b'abc')
  assert read_saved(params, 'K', 16) == (b'abc', True)
  monkeypatch.setattr(saved_source.os, 'open', lambda *a, **k: (_ for _ in ()).throw(PermissionError()))
  assert read_saved(params, 'K', 16) == (b'', False)


def test_path_lookup_is_remembered_per_params_object(params, monkeypatch):
  keep_unchanged_reads()
  write(params, 'K', b'abc')
  calls = []
  real = params.get_param_path
  params.get_param_path = lambda key: calls.append(key) or real(key)
  for _ in range(3):
    assert read_saved(params, 'K', 16) == (b'abc', True)
  assert calls == ['K']
  other = FakeParams(params.root)
  other.get_param_path = lambda key: calls.append(('other', key)) or real(key)
  assert read_saved(other, 'K', 16) == (b'abc', True)
  assert calls == ['K', ('other', 'K')]
