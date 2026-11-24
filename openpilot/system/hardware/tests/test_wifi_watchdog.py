from openpilot.system.hardware.wifi_watchdog import COOLDOWN_S, STUCK_S, NetworkStrength, NetworkType, WifiWatchdog, default_kick

WIFI, CELL, NONE = NetworkType.wifi, NetworkType.cell4G, NetworkType.none
LOST, GOOD = NetworkStrength.unknown, NetworkStrength.good


def make(lost=True):
  calls = []
  wd = WifiWatchdog(kick=lambda: calls.append('kick') or True, signal_lost=lambda: lost, threaded=False)
  return wd, calls


def feed(wd, samples, step=10.0, t0=0.0):
  t = t0
  for net, strength in samples:
    wd.update(net, strength, t)
    t += step
  return t


def test_the_42_minute_stuck_wifi_is_dropped_after_a_minute():
  wd, calls = make()
  # good signal at the start, then the network is gone but stays the default route (the 11-24 drive)
  t = feed(wd, [(WIFI, GOOD)] * 3 + [(WIFI, LOST)] * 6)
  assert calls == []                      # 50 s of no signal: not yet
  wd.update(WIFI, LOST, t)                # 60 s
  assert calls == ['kick']


def test_one_kick_per_cooldown_while_still_stuck():
  wd, calls = make()
  t = feed(wd, [(WIFI, LOST)] * int((STUCK_S + COOLDOWN_S) / 10 + 2))
  assert calls == ['kick', 'kick']
  assert wd.kicks == 2 and t > STUCK_S + COOLDOWN_S


def test_signal_coming_back_resets_the_clock():
  wd, calls = make()
  feed(wd, ([(WIFI, LOST)] * 5 + [(WIFI, GOOD)]) * 10)
  assert calls == []


def test_not_wifi_never_kicks():
  # no network at all (hotspot off) and the phone uplink both read as something other than Wi-Fi
  wd, calls = make()
  feed(wd, [(NONE, LOST)] * 50 + [(CELL, LOST)] * 50)
  assert calls == []


def test_a_busy_cpu_timeout_is_not_a_lost_network():
  # hardwared's quick 0.2 s poll can time out under load; the slower re-poll finds the signal, so nothing is dropped
  wd, calls = make(lost=False)
  feed(wd, [(WIFI, LOST)] * 10)
  assert calls == [] and wd.kicks == 1    # it looked once, then waits out the cooldown


def test_kick_disconnects_then_turns_autoconnect_back_on():
  ran = []
  assert default_kick(lambda cmd: ran.append(cmd) or True)
  assert ran[0][-3:] == ['device', 'disconnect', 'wlan0']
  assert ran[1] == ['nmcli', 'device', 'set', 'wlan0', 'autoconnect', 'yes']


def test_autoconnect_is_restored_even_if_the_disconnect_fails():
  ran = []
  assert not default_kick(lambda cmd: ran.append(cmd) or cmd[1] == 'device' and cmd[2] == 'set')
  assert len(ran) == 2
