"""BlueDriver protocol and decoding, checked against replies captured from the user's car (2026-11-12)."""
import asyncio
import json

import pytest

from openpilot.starpilot.obd import bluedriver as bd
from openpilot.starpilot.obd import bluedriverd


def test_xor_codec_keeps_cr_and_prompt_and_round_trips():
  assert bd.encode_command('LMI') == bytes.fromhex('6a6b6f0d')  # as seen on the air
  assert bd.xor(bd.xor(b'0144\r>')) == b'0144\r>'
  assert bd.xor(b'\r>') == b'\r>'
  with pytest.raises(ValueError):
    bd.encode_command('01\n0C')


def test_reassembler_joins_fragments_at_the_prompt():
  r = bd.Reassembler()
  wire = bd.xor(b'OSAPI v3.16\rOK\r>')
  assert r.feed(wire[:5]) is None
  assert r.feed(wire[5:]) == 'OSAPI v3.16\nOK'
  assert r.feed(bd.xor(b'15 C4 \r>')) == '15 C4'


@pytest.mark.parametrize('code, reply, expected', [
  ('010C', '15 C4', 1393.0),  # engine idling to warm up
  ('0110', '02 AA', 6.82),
  ('0144', '7E E0', 0.991),
  ('0144', '83 E0', 1.030),  # engine off, as the BlueDriver app showed
  ('0106', '7A', -4.69),
  ('0107', '7C', -3.13),
  ('0162', '8F', 18.0),
  ('0163', '01 2C', 300.0),  # 221 ft-lb reference torque
  ('015B', '80', 50.2),
  ('015B', '88', 53.33),
  ('01A6', '00 02 39 9E', 14582.2),
  ('0105', '4C', 36.0),
  ('0142', '35 C4', 13.764),
])
def test_decodes_real_replies(code, reply, expected):
  assert bd.decode(code, reply) == pytest.approx(expected, abs=0.01)


@pytest.mark.parametrize('reply', [None, '', 'NO DATA', '?', 'SEARCHING...', 'CAN ERROR', '15', '15 C4 00', 'ZZ'])
def test_errors_and_wrong_sizes_decode_to_none(reply):
  assert bd.decode('010C', reply) is None


def test_fuel_rate():
  assert bd.fuel_rate_gps(6.82, 0.991, 1393) == pytest.approx(6.82 / (bd.STOICH_AFR * 0.991))
  assert bd.fuel_rate_gps(0.0, 1.03, 0.0) == 0.0  # engine off (EV)
  assert bd.fuel_rate_gps(3.0, 1.0, 50) == 0.0  # not turning
  assert bd.fuel_rate_gps(None, 1.0, 1500) is None
  assert bd.fuel_rate_gps(500.0, 1.0, 1500) is None  # impossible flow


def test_fuel_integrator_never_bridges_gaps():
  f = bd.FuelIntegrator()
  for i in range(11):
    f.add(i * 0.1, 0.5)
  assert f.grams == pytest.approx(0.5) and f.seconds == pytest.approx(1.0)
  f.add(1.1, None)  # link dropped
  f.add(5.0, 0.5)
  f.add(5.1, 0.5)
  assert f.grams == pytest.approx(0.55)
  f.add(9.0, 0.5)  # a long gap without a None is not bridged either
  assert f.grams == pytest.approx(0.55)


def test_poll_plan_reads_fuel_every_cycle():
  for cycle in range(20):
    assert bd.plan(cycle)[:3] == list(bd.FAST)
  assert '0163' in bd.plan(0) and '0163' not in bd.plan(1)


class FakeLink:
  """Answers like the scanner did with the engine idling."""
  REPLIES = {'010C': '15 C4', '0110': '02 AA', '0144': '7E E0', '015B': '80', '0162': '8F', '0106': '7A', '0107': '7C',
             '0104': '4B', '0105': '4C', '0146': '2F', '0142': '35 C4', '01A6': '00 02 39 9E', '0163': '01 2C', '012F': '7A',
             '010F': '40', '019D': '00 10 00 10', '019A': '9A 07 00 3B AB'}

  def __init__(self, clock, fail_after=None):
    self.clock, self.count, self.fail_after = clock, 0, fail_after

  async def query(self, code, wait_s=1.0):
    self.count += 1
    self.clock[0] += 0.04
    if self.fail_after is not None and self.count > self.fail_after:
      return None
    return self.REPLIES[code]


def test_poll_integrates_fuel_and_writes_live_and_csv(tmp_path):
  clock = [100.0]
  session = bluedriverd.Session(tmp_path, 'drive1', clock=lambda: clock[0])
  session.connected = True
  link = FakeLink(clock)
  asyncio.run(bluedriverd.poll(link, session, clock=lambda: clock[0], should_run=lambda: link.count < 200))
  rate = bd.fuel_rate_gps(bd.decode('0110', '02 AA'), bd.decode('0144', '7E E0'), 1393.0)
  assert session.fuel.grams == pytest.approx(rate * session.fuel.seconds, rel=1e-6)
  assert session.fuel.seconds > 5.0
  live = bluedriverd.read_live(tmp_path, now=clock[0])
  assert live['connected'] and live['soc'] == pytest.approx(50.2, abs=0.01) and live['fuel_g'] > 0
  session.close()
  rows = (tmp_path / 'drives' / 'drive1.csv').read_text().splitlines()
  assert rows[0].startswith('t,rpm,maf') and len(rows) > 3


def test_poll_gives_up_when_the_scanner_stops_answering(tmp_path):
  clock = [100.0]
  session = bluedriverd.Session(tmp_path, 'drive1', clock=lambda: clock[0])
  link = FakeLink(clock, fail_after=20)
  with pytest.raises(ConnectionError):
    asyncio.run(bluedriverd.poll(link, session, clock=lambda: clock[0]))
  assert link.count == 30
  session.close()


def test_read_live_rejects_stale_and_malformed(tmp_path):
  (tmp_path / 'live.json').write_text(json.dumps({'version': 1, 't': 10.0, 'connected': True}))
  assert bluedriverd.read_live(tmp_path, now=11.0) is not None
  assert bluedriverd.read_live(tmp_path, now=20.0) is None
  (tmp_path / 'live.json').write_text('{oops')
  assert bluedriverd.read_live(tmp_path, now=11.0) is None


def test_enabled_needs_explicit_opt_in(tmp_path):
  assert not bluedriverd.enabled(tmp_path)
  (tmp_path / 'config.json').write_text('{"enabled": true}')
  assert bluedriverd.enabled(tmp_path)
  (tmp_path / 'config.json').write_text('{"enabled": "yes"}')
  assert not bluedriverd.enabled(tmp_path)


class V:
  def __init__(self, value):
    self.value = value


def test_bonded_scanner_detection():
  assert bluedriverd.is_bonded_scanner({'Paired': V(True), 'Name': V('BlueDriv')})
  assert bluedriverd.is_bonded_scanner({'Paired': V(True), 'Alias': V('LSB3-180DF976BFAF')})
  assert not bluedriverd.is_bonded_scanner({'Paired': V(False), 'Name': V('BlueDriv')})
  assert not bluedriverd.is_bonded_scanner({'Paired': V(True), 'Name': V('Galaxy S26')})
  assert not bluedriverd.is_bonded_scanner(None)


def test_monitor_status_sums_every_ecu():
  assert bd.monitor_status('00 07 E5 00') == (False, 0)
  assert bd.monitor_status('82 07 E5 00\n01 00 00 00') == (True, 3)
  assert bd.monitor_status('NO DATA') is None


@pytest.mark.parametrize('reply, codes', [
  ('00', []),
  ('01 04 20', ['P0420']),
  ('43 01 04 20', ['P0420']),
  ('02 04 20 C1 23', ['P0420', 'U0123']),
  ('NO DATA', []),
])
def test_trouble_codes(reply, codes):
  assert bd.trouble_codes(reply) == codes


def test_fuel_level_and_intake_decode():
  assert bd.decode('012F', '7A') == pytest.approx(47.84, abs=0.01)
  assert bd.decode('010F', '40') == 24.0


def test_health_check_writes_codes_without_clearing(tmp_path):
  class Link:
    sent = []

    async def query(self, code, wait_s=1.0):
      self.sent.append(code)
      return {'0101': '81 07 E5 00', '03': '01 04 20', '07': '00'}[code]
  link = Link()
  doc = asyncio.run(bluedriverd.health_check(link, tmp_path, 'd1'))
  assert doc['mil'] is True and doc['dtc_count'] == 1 and doc['stored'] == ['P0420'] and doc['pending'] == []
  assert json.loads((tmp_path / 'health.json').read_text())['stored'] == ['P0420']
  assert '04' not in link.sent  # mode 04 would clear codes; never sent


def test_direct_fuel_rate_and_battery_voltage_from_this_cars_replies():
  # 11-14 probe on the Elantra Hybrid, engine idling
  assert bd.decode('019D', '00 10 00 10') == pytest.approx(0.32)
  assert bd.decode('019D', '9D 00 10 00 10') == pytest.approx(0.32)   # PID echoed in front
  assert bd.decode('019D', 'FF FF 00 00') is None                    # 1310 g/s: not a real reading
  assert bd.decode('019D', 'NO DATA') is None
  assert bd.decode('019A', '9A 07 00 3B AB') == pytest.approx(238.67, abs=0.01)
  assert bd.decode('019A', '07 00 3B AB') == pytest.approx(238.67, abs=0.01)
  assert bd.decode('019A', '07 00 00 10') is None                    # 0.25 V: outside a hybrid pack's range
  assert bd.decode('019A', '07 00') is None


def test_direct_fuel_is_polled_every_cycle_and_voltage_every_fifth():
  assert all('019D' in bd.plan(c) for c in range(10))
  assert ['019A' in bd.plan(c) for c in range(5)] == [True, False, False, False, False]


def test_session_integrates_both_fuel_figures_and_never_bridges_a_missing_answer(tmp_path):
  from openpilot.starpilot.obd.bluedriverd import Session
  s = Session(tmp_path, 'drive')
  for i in range(11):
    s.update('019D', 0.32, i * 0.1)
  assert s.fuel_direct.grams == pytest.approx(0.32, abs=1e-6)
  s.update('019D', None, 1.1)
  s.update('019D', 0.32, 1.2)
  s.update('019D', 0.32, 1.3)
  assert s.fuel_direct.grams == pytest.approx(0.352, abs=1e-6)        # the gap was not integrated
  assert s.live_document(2.0)['fuel_direct_g'] == pytest.approx(0.352, abs=1e-3)
