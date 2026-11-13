import stat
import threading
import time

from openpilot.starpilot.galaxy import remote_service


class FakePairing:
  def __init__(self, root):
    self.root = root

  def read(self):
    return {'slug': 'abcdefghABCDEFGH'}

  @staticmethod
  def frpc_config(slug, remote_port, auth_port):
    return f'# {slug} {remote_port} {auth_port}\n'


def fake_frpc(tmp_path):
  log = tmp_path / 'starts.log'
  binary = tmp_path / 'frpc'
  binary.write_text(f'#!/bin/sh\necho start >> {log}\nexec sleep 30\n')
  binary.chmod(binary.stat().st_mode | stat.S_IXUSR)
  return binary, log


def starts(log):
  return len(log.read_text().splitlines()) if log.exists() else 0


def wait_for(condition, timeout=6.0):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if condition():
      return True
    time.sleep(0.05)
  return False


def test_tunnel_restarts_when_the_network_changes(tmp_path, monkeypatch):
  binary, log = fake_frpc(tmp_path)
  monkeypatch.setenv('STARPILOT_FRPC_BIN', str(binary))
  route = {'value': None}
  stop = threading.Event()
  worker = threading.Thread(target=remote_service.supervise_tunnel,
                            args=(stop, FakePairing(tmp_path)), kwargs={'route': lambda: route['value']}, daemon=True)
  worker.start()
  try:
    assert wait_for(lambda: starts(log) == 1)  # starts even before there is a network, as before
    route['value'] = '192.168.1.157'           # internet arrives: restart at once instead of waiting on keepalive
    assert wait_for(lambda: starts(log) == 2)
    time.sleep(2.5)
    assert starts(log) == 2                    # a steady network leaves the tunnel alone
    route['value'] = None                      # losing the route alone does not churn the tunnel
    time.sleep(1.5)
    assert starts(log) == 2
    route['value'] = '172.20.10.3'             # Wi-Fi to hotspot
    assert wait_for(lambda: starts(log) == 3)
  finally:
    stop.set()
    worker.join(timeout=10)
  assert not worker.is_alive()
  assert not any(p.name.startswith('.frpc-') for p in tmp_path.iterdir())  # config files cleaned up


def test_network_route_needs_no_traffic():
  value = remote_service.network_route()
  assert value is None or value.count('.') == 3


def test_frpc_config_keeps_relay_settings():
  from openpilot.starpilot.galaxy.remote import RemotePairing
  config = RemotePairing.frpc_config('abcdefghABCDEFGH', 8084, 8083)
  assert 'tcpMuxKeepaliveInterval = 10' in config
  assert 'dialServerTimeout = 5' in config
