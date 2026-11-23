import os
from types import SimpleNamespace

from openpilot.starpilot.power import chestnut_idle as ci
from openpilot.starpilot.power.chestnut_idle import CAR_OFF_GRACE_S, IDLE_S, ChestnutIdle


def run(idle, start, end, step=5., **kw):
  fired = []
  t = start
  while t <= end:
    if idle.update(t, **kw):
      fired.append(t)
      break   # the link drops once switched off
    t += step
  return fired


def test_retries_every_idle_period_if_the_link_stays_up():
  idle = ChestnutIdle()
  fired = [t for t in range(0, 1000, 5) if idle.update(float(t), started=False, in_use=False, link_up=True)]
  assert fired == [IDLE_S, 2 * IDLE_S + 5, 3 * IDLE_S + 10]


def test_never_switches_off_while_driving():
  idle = ChestnutIdle()
  assert run(idle, 0, 3600, started=True, in_use=False, link_up=True) == []


def test_car_off_switches_off_once_modeld_lets_go():
  idle = ChestnutIdle()
  idle.update(0, started=True, in_use=True, link_up=True)
  assert run(idle, 5, 20, started=False, in_use=True, link_up=True) == []   # modeld still closing
  fired = run(idle, 25, 200, started=False, in_use=False, link_up=True)
  assert fired == [25 + CAR_OFF_GRACE_S]


def test_test_scripts_wake_it_and_it_goes_back_off_after_5_idle_minutes():
  idle = ChestnutIdle()
  assert run(idle, 0, 60, started=False, in_use=False, link_up=False) == []  # off, parked
  assert run(idle, 65, 1000, started=False, in_use=True, link_up=True) == []  # a long test holds it
  fired = run(idle, 1005, 2000, started=False, in_use=False, link_up=True)
  assert fired == [1005 + IDLE_S]   # the 5 minute rule, not the car-off grace


def test_powered_on_at_boot_with_nothing_using_it_goes_off_after_5_minutes():
  idle = ChestnutIdle()
  assert run(idle, 0, 1000, started=False, in_use=False, link_up=True) == [IDLE_S]


def test_use_restarts_the_idle_clock_and_a_backwards_clock_never_fires_early():
  idle = ChestnutIdle()
  idle.update(0, started=False, in_use=False, link_up=True)
  assert not idle.update(IDLE_S - 1, started=False, in_use=True, link_up=True)
  assert not idle.update(IDLE_S + 1, started=False, in_use=False, link_up=True)
  assert not idle.update(10, started=False, in_use=False, link_up=True)
  assert idle.update(10 + IDLE_S, started=False, in_use=False, link_up=True)


def test_other_users_sees_another_process_holding_the_node(tmp_path):
  node = '/dev/bus/usb/004/002'
  for pid, target in ((100, node), (200, '/dev/null')):
    (tmp_path / str(pid) / 'fd').mkdir(parents=True)
    os.symlink(target, tmp_path / str(pid) / 'fd' / '7')
  (tmp_path / 'self').mkdir()
  assert ci.other_users(node, own_pid=1, proc=str(tmp_path))
  assert not ci.other_users(node, own_pid=100, proc=str(tmp_path))   # our own handle does not count
  assert not ci.other_users('/dev/bus/usb/004/003', own_pid=1, proc=str(tmp_path))


def test_node_path_and_kill_switch(tmp_path):
  assert ci.node_path({'busnum': 4, 'devnum': 2}) == '/dev/bus/usb/004/002'
  assert ci.node_path({'busnum': None, 'devnum': 2}) is None
  f = tmp_path / 'chestnut_idle.json'
  assert ci.enabled(str(f))
  f.write_text('{"mode": "off"}')
  assert not ci.enabled(str(f))
  f.write_text('garbage')
  assert ci.enabled(str(f))


class Handle:
  def __init__(self): self.writes = []
  def controlWrite(self, *args, **kwargs): self.writes.append(args)


class SM(dict):
  def __init__(self, started):
    super().__init__(deviceState=SimpleNamespace(started=bool(started)))
    self.seen = {'deviceState': started is not None}


def test_monitor_hook_sends_the_switch_only_when_parked_and_unused(monkeypatch):
  from openpilot.system.hardware.chestnut import monitoring
  monkeypatch.setattr(ci, 'enabled', lambda path=ci.SETTINGS_PATH: True)
  monkeypatch.setattr(ci, 'other_users', lambda node, own_pid=None, proc='/proc': False)
  h, idle = Handle(), ChestnutIdle()
  for t in range(0, 400, 5):   # unknown drive state counts as driving
    monitoring.idle_power_off(h, idle, SM(None), '/dev/bus/usb/004/002', ci.LINK_UP, float(t))
  assert h.writes == []
  for t in range(400, 400 + int(IDLE_S) + 5, 5):
    monitoring.idle_power_off(h, idle, SM(False), '/dev/bus/usb/004/002', ci.LINK_UP, float(t))
  assert h.writes == [(0x40, 0xF3, 0, 0, b'')]
  monitoring.idle_power_off(h, idle, SM(False), '/dev/bus/usb/004/002', 0x0, 2000.)   # already off
  assert len(h.writes) == 1
