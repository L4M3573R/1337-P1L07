import json
import math

from openpilot.starpilot.power.cpu_boost import (BIG_POLICY, GUARD_COOL_C, GUARD_HOT_C, LITTLE_POLICY, STOCK_KHZ, CpuBoost,
                                                 pick_step, read_config)

# Snapdragon 845 tables (the comma's real ones are read from scaling_available_frequencies)
LITTLE = [300000, 576000, 748800, 1132800, 1478400, 1689600, 1766400]
BIG = [825600, 1209600, 1689600, 1766400, 1843200, 1920000, 1996800, 2092800, 2169600, 2246400, 2323200, 2400000,
       2476800, 2553600, 2649600, 2745600, 2803200]
STEPS = {LITTLE_POLICY: LITTLE, BIG_POLICY: BIG}


class Sysfs:
  def __init__(self):
    self.cap = {LITTLE_POLICY: STOCK_KHZ, BIG_POLICY: STOCK_KHZ}
    self.writes = []

  def write(self, value, path):
    policy, name = path.rsplit('/', 1)
    assert name == 'scaling_max_freq' and policy in self.cap
    self.cap[policy] = int(value)
    self.writes.append((policy, int(value)))


def boost(tmp_path, config=None, steps=STEPS):
  path = tmp_path / 'cpu_boost.json'
  if config is not None:
    path.write_text(json.dumps(config))
  fs = Sysfs()
  return CpuBoost(path, fs.write, steps=steps, read=lambda policy: fs.cap[policy]), fs, path


BOOSTED = {LITTLE_POLICY: 1766400, BIG_POLICY: 2169600}
STOCK = {LITTLE_POLICY: STOCK_KHZ, BIG_POLICY: STOCK_KHZ}


def test_picks_the_highest_step_at_or_below_the_target_and_never_below_stock():
  assert pick_step(BIG, 2200) == 2169600
  assert pick_step(BIG, 2247) == 2246400
  assert pick_step(BIG, 1000) == STOCK_KHZ
  assert pick_step(LITTLE, math.inf) == 1766400
  assert pick_step([], 2200) == STOCK_KHZ  # unknown table: stay stock


def test_config_defaults_on_and_rejects_garbage(tmp_path):
  c = read_config(tmp_path / 'missing.json')
  assert (c.on, c.big_mhz, c.little_max) == (True, 2200, True)
  (tmp_path / 'bad.json').write_text('{nope')
  assert read_config(tmp_path / 'bad.json').on
  (tmp_path / 'off.json').write_text('{"mode": "off"}')
  assert not read_config(tmp_path / 'off.json').on
  (tmp_path / 'big.json').write_text('{"big_mhz": 99999}')
  assert read_config(tmp_path / 'big.json').big_mhz == 2900
  (tmp_path / 'nan.json').write_text('{"big_mhz": "fast", "little": "stock"}')
  c = read_config(tmp_path / 'nan.json')
  assert c.big_mhz == 2200 and not c.little_max


def test_boosts_both_clusters_with_ignition_and_writes_only_on_change(tmp_path):
  b, fs, _ = boost(tmp_path)
  assert b.update(True, True, 60.0, now=0.0) == BOOSTED
  for t in range(1, 20):
    assert b.update(True, True, 60.0, now=t * 0.5) == {}
  assert len(fs.writes) == 2 and b.boosting and b.fan_boost(True)


def test_little_can_stay_stock(tmp_path):
  b, _, _ = boost(tmp_path, {"little": "stock"})
  assert b.update(True, True, 60.0, now=0.0) == {LITTLE_POLICY: STOCK_KHZ, BIG_POLICY: 2169600}


def test_ignition_off_puts_the_stock_caps_back(tmp_path):
  b, fs, _ = boost(tmp_path)
  b.update(True, True, 60.0, now=0.0)
  assert b.update(False, True, 60.0, now=0.5) == STOCK
  assert not b.boosting and not b.fan_boost(False)


def test_power_save_leaves_the_offline_big_cluster_alone_and_reset_rewrites(tmp_path):
  b, fs, _ = boost(tmp_path)
  b.update(True, True, 60.0, now=0.0)
  assert b.update(False, False, 60.0, now=0.5) == {LITTLE_POLICY: STOCK_KHZ}  # big cores offline: never written
  b.update(True, True, 60.0, now=1.0)
  fs.cap = dict(STOCK)  # set_power_save wrote stock again
  b.reset()
  assert b.update(True, True, 60.0, now=1.5) == BOOSTED


def test_heat_guard_drops_to_stock_and_returns_only_when_cool(tmp_path):
  b, fs, _ = boost(tmp_path)
  b.update(True, True, 70.0, now=0.0)
  assert b.update(True, True, GUARD_HOT_C, now=0.5) == STOCK
  assert b.fan_boost(True)  # fan stays on the harder curve while hot
  assert b.update(True, True, GUARD_COOL_C + 1, now=1.0) == {}  # hysteresis
  assert b.update(True, True, GUARD_COOL_C - 1, now=1.5) == BOOSTED
  assert b.update(True, True, math.nan, now=2.0) == {}  # a bad reading changes nothing


def test_switch_file_flips_it_during_a_drive(tmp_path):
  b, fs, path = boost(tmp_path, {"mode": "on", "big_mhz": 2400})
  assert b.update(True, True, 60.0, now=0.0) == {LITTLE_POLICY: 1766400, BIG_POLICY: 2400000}
  path.write_text('{"mode": "off"}')
  assert b.update(True, True, 60.0, now=1.0) == {}  # not re-read yet
  assert b.update(True, True, 60.0, now=5.0) == STOCK
  assert not b.fan_boost(True)


def test_someone_else_rewriting_a_cap_is_put_back(tmp_path):
  b, fs, _ = boost(tmp_path)
  b.update(True, True, 60.0, now=0.0)
  fs.cap[LITTLE_POLICY] = STOCK_KHZ
  assert b.update(True, True, 60.0, now=5.0) == {LITTLE_POLICY: 1766400}


def test_unknown_tables_stay_stock(tmp_path):
  b, fs, _ = boost(tmp_path, steps={})
  assert b.update(True, True, 60.0, now=0.0) == STOCK
