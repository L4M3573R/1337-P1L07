"""Supervise the original Galaxy FRPC route only while a private pairing exists."""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import threading
from collections.abc import Callable

from openpilot.starpilot.galaxy.remote import RemotePairing
from openpilot.starpilot.galaxy.frpc_asset import bundled_frpc_path


def frpc_binary() -> str | None:
  override = os.environ.get('STARPILOT_FRPC_BIN')
  if override:
    return override if Path(override).is_file() and os.access(override, os.X_OK) else None
  return bundled_frpc_path() or shutil.which('frpc')


def network_route() -> str | None:
  """The local address used to reach the internet, or None without a route. Sends no packets."""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
      probe.connect(('1.1.1.1', 53))
      return probe.getsockname()[0]
  except OSError:
    return None


def supervise_tunnel(stop: threading.Event, pairing: RemotePairing, *, remote_port: int = 8084, auth_port: int = 8083,
                     route: Callable[[], str | None] = network_route) -> None:
  process = None
  active_slug = None
  active_route = None
  config_path = None
  try:
    while not stop.wait(1):
      record = pairing.read()
      slug = record['slug'] if record else None
      current_route = route()
      # A new network (Wi-Fi to hotspot, or the first connection after boot) leaves the old relay connection
      # dead until its keepalive times out; restart the tunnel at once so Galaxy is reachable as soon as the
      # internet is.
      moved = current_route is not None and current_route != active_route
      if process is not None and (slug != active_slug or moved or process.poll() is not None):
        if process.poll() is None:
          process.terminate()
          try:
            process.wait(timeout=5)
          except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
        process = None
        active_slug = None
        if config_path is not None:
          Path(config_path).unlink(missing_ok=True)
          config_path = None
      binary = frpc_binary() if slug and process is None else None
      if slug and binary and process is None:
        fd, config_path = tempfile.mkstemp(prefix='.frpc-', suffix='.toml', dir=pairing.root)
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
          stream.write(pairing.frpc_config(slug, remote_port, auth_port))
        process = subprocess.Popen([binary, '-c', config_path], stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        active_slug = slug
        active_route = current_route
  finally:
    if process is not None and process.poll() is None:
      process.terminate()
      try:
        process.wait(timeout=5)
      except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)
    if config_path is not None:
      Path(config_path).unlink(missing_ok=True)
