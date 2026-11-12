"""Real fuel from the dash average, hybrid power flow, and Pulse and Glide in the energy ledger."""
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.efficiency import store, tank
from openpilot.starpilot.efficiency.efficiencyd import Recorder, pulse_glide_phase
from openpilot.starpilot.efficiency.hybrid import Cluster, PowerFlow, decode_cluster, decode_power_flow, scan
from openpilot.starpilot.efficiency.ledger import Ledger, Sample, summarize

MPH = 0.44704


def cluster_frame(avg_mpg, range_mi):
  return (round(avg_mpg * 10) << 6 | range_mi << 24).to_bytes(8, 'little')


def test_cluster_and_power_flow_decode_like_the_recorded_drives():
  # Values seen on route 4c: dash average 49.8 mpg, range 491 mi; power flow 00 72 d3 ff 00 10 00 00.
  assert decode_cluster(cluster_frame(49.8, 491)) == Cluster(pytest.approx(49.8), 491)
  flow = decode_power_flow(bytes.fromhex('0072d3ff00100000'))
  assert (flow.mode, flow.flow, flow.meter, flow.charge) == (16, 'stopped', 211, 114)
  assert decode_power_flow(bytes([0, 110, 222, 0, 0, 96, 0, 0])).flow == 'regen'
  assert decode_power_flow(bytes([0, 110, 100, 0, 0, 48, 0, 0])).flow == 'engine'
  assert decode_power_flow(bytes([0, 110, 200, 0, 0, 32, 0, 0])).flow == 'ev'
  assert decode_power_flow(b'\x00') is None and decode_cluster(b'\x00' * 6) is None


def test_scan_takes_the_latest_bus0_readings_only():
  frames = [(0x595, bytes([0, 1, 2, 0, 0, 32, 0, 0]), 0), (0x595, bytes([0, 1, 3, 0, 0, 96, 0, 0]), 0),
            (0x50C, cluster_frame(55.0, 400), 2), (0x50C, cluster_frame(50.0, 410), 0)]
  flow, cl = scan(frames)
  assert flow.flow == 'regen' and cl == Cluster(pytest.approx(50.0), 410)


def test_tank_detects_a_fill_up_from_the_range_jump_and_counts_miles_across_drives(tmp_path):
  state = tank.TankState()
  assert state.gallons_used(50.0) is None  # unknown until a fill-up is seen or seeded
  state.observe_range(80)
  state.drive(10.0)
  assert state.gallons_used(50.0) is None
  assert state.observe_range(491)  # filled
  state.drive(25.0)
  assert state.gallons_used(50.0) == pytest.approx(0.5)
  assert not state.observe_range(485)  # range only falls (or rises a little) while driving
  tank.save(state, tmp_path / 't.json')
  again = tank.load(tmp_path / 't.json')
  assert again.miles_since_fill == pytest.approx(25.0) and again.fills_seen == 1 and again.last_range_mi == 485


def test_corrupt_tank_file_starts_unknown(tmp_path):
  (tmp_path / 't.json').write_text('{nope')
  assert tank.load(tmp_path / 't.json').miles_since_fill is None


def sample(t, speed=20.0, fuel=None, flow=None, pg=None, accel=0.0, pitch=0.0, meter=None):
  return Sample(t=t, speed=speed, accel=accel, pitch=pitch, mass=1450.0, long_active=True, fuel_gal=fuel, flow=flow,
                power_meter=meter, pulse_glide=pg)


def test_stretch_fuel_is_bridged_between_stretches_so_no_fuel_is_lost():
  ledger = Ledger()
  fuel = 1.0
  for i in range(1200):  # 60 s at 20 Hz and 20 m/s: several 250 m stretches, the dash updates every 10 s
    if i % 200 == 0:
      fuel = 1.0 + i * 0.00005
    ledger.add(sample(i * 0.05, fuel=fuel))
  total, _ = summarize(ledger.finish())
  assert total.fuel_gal == pytest.approx(fuel - 1.0, abs=1e-9)  # every update counted exactly once
  assert total.fuel_distance == pytest.approx(total.distance)


def test_real_mpg_and_power_flow_time_per_mode():
  ledger = Ledger()
  for i in range(2400):  # 120 s at 20 m/s = 2.4 km
    flow = 'regen' if i < 1200 else 'engine'
    ledger.add(sample(i * 0.05, fuel=i * 0.05 * 20 / 1609.344 / 50.0, flow=flow, pg='glide' if i < 1200 else 'pulse'))
  total, contexts = summarize(ledger.finish())
  assert total.mpg == pytest.approx(50.0, rel=0.01)
  assert total.regen_s == pytest.approx(60.0, abs=0.2) and total.engine_s == pytest.approx(60.0, abs=0.2)
  assert {k[4] for k in contexts} == {'pulse-glide'}  # Pulse and Glide is its own mode


def test_flow_histogram_records_where_the_hybrid_stays_electric():
  ledger = Ledger()
  for i in range(100):
    ledger.add(sample(i * 0.05, speed=61 * MPH, flow='ev', pg='glide', accel=-0.03, meter=207))
  key = next(iter(ledger.flow_hist))
  speed, accel, phase = key.split('|')
  assert (speed, phase) == ('60', 'glide') and float(accel) == pytest.approx(-0.05)
  n, ev, regen, engine, meter = ledger.flow_hist[key]
  assert n >= 99 and ev == n and regen == engine == 0 and meter / n == pytest.approx(207)


def car_state(speed=20.0):
  return NS(vEgo=speed, aEgo=0.0, standstill=False)


def test_recorder_counts_real_fuel_from_the_dash_and_carries_miles(tmp_path):
  tank.save(tank.TankState(miles_since_fill=10.0, last_range_mi=400), tmp_path / 'tank.json')
  rec = Recorder(1450.0, 'drive', tmp_path)
  t = 0.0
  engaged = NS(longActive=True, orientationNED=[0.0, 0.0, 0.0])
  for _ in range(1200):  # 60 s at 20 m/s
    rec.observe_can(PowerFlow(96, 'regen', 222, 110), Cluster(50.0, 400))  # steady 50 mpg average all drive
    rec.observe(t, car_state(), engaged, pulse_glide='glide')
    t += 0.05
  rec.finish()
  assert tank.load(tmp_path / 'tank.json').miles_since_fill == pytest.approx(10.0 + 20 * 60 / 1609.344, rel=0.01)
  total = store.load_totals(tmp_path)[0]
  assert total.fuel_gal == pytest.approx(20 * 60 / 1609.344 / 50.0, rel=0.02)  # a steady average still counts fuel
  assert total.regen_s > 55
  assert store.load_flow(tmp_path)  # the power-flow records were saved with the drive


def test_fill_up_resets_the_counter_without_bridging_fuel_across_it(tmp_path):
  rec = Recorder(1450.0, 'drive', tmp_path)
  rec.observe_can(None, Cluster(45.0, 60))
  assert rec.fuel_gal is None  # miles since fill-up still unknown
  rec.observe_can(None, Cluster(49.8, 491))  # range jumped: filled
  assert rec.tank.miles_since_fill == 0.0 and rec.fuel_gal == pytest.approx(0.0)


def test_pulse_glide_phase_only_when_enabled():
  sm = {'starpilotLongitudinalPlan': NS(pulseGlideEnabled=True, pulseGlidePhase='glide')}
  seen = NS(seen={'starpilotLongitudinalPlan': True}, __getitem__=None)

  class SM(dict):
    seen = {'starpilotLongitudinalPlan': True}
  assert pulse_glide_phase(SM(sm)) == 'glide'
  sm['starpilotLongitudinalPlan'] = NS(pulseGlideEnabled=False, pulseGlidePhase='off')
  assert pulse_glide_phase(SM(sm)) is None
  assert seen is not None


def test_a_dash_refresh_correction_is_kept_but_a_fill_up_is_not_bridged():
  ledger = Ledger()
  values = [1.000, 1.010, 1.020, 1.015, 1.030, 0.000, 0.010]  # small correction at index 3, fill-up at index 5
  for i, fuel in enumerate(values):
    for k in range(60):
      ledger.add(sample(i * 3.0 + k * 0.05, fuel=fuel))
  total, _ = summarize(ledger.finish())
  assert total.fuel_gal == pytest.approx((1.030 - 1.000) + (0.010 - 0.000), abs=1e-9)


def test_report_sections_show_real_mpg_by_mode_and_the_ev_cutoff():
  from openpilot.starpilot.efficiency.ledger import Totals
  from openpilot.starpilot.efficiency.report import ev_cutoff_section, real_fuel_section
  pg = Totals(distance=3218.7, fuel_gal=0.05, fuel_distance=3218.7, ev_s=10, regen_s=10, engine_s=80)
  steady = Totals(distance=1609.3, fuel_gal=0.0275, fuel_distance=1609.3, engine_s=60)
  text = "\n".join(real_fuel_section({('60+ mph', 'flat', 'straight', 'open road', 'pulse-glide'): pg,
                                      ('60+ mph', 'flat', 'straight', 'open road', 'chill'): steady}))
  assert "pulse-glide 40.0" in text and "chill 36.4" in text
  flow = {'60|-0.05|glide': [200, 150, 44, 6, 42000.0], '64|-0.05|glide': [200, 10, 14, 176, 42000.0],
          '64|-0.40|glide': [500, 0, 500, 0, 110000.0]}  # the strong regen glide is outside the feather zone
  cutoff = "\n".join(ev_cutoff_section(flow))
  assert "60-62 mph" in cutoff and "electric  97%" in cutoff and "64-66 mph" in cutoff and "electric  12%" in cutoff
