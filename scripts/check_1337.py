#!/usr/bin/env python3
"""Verify every 1337-P1L07 customization is still present and working after an upstream (StarPilot Dom) merge.

  .venv/bin/python scripts/check_1337.py            # anchors only: each feature's key code is still there
  .venv/bin/python scripts/check_1337.py --tests    # anchors, then each feature's tests through ./dev

The feature list mirrors docs/1337_FORK_CHANGES.md. When a feature is added, add it here and there.
Exit status is non-zero when any anchor is missing or any test run fails.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OP = 'openpilot/'
STARPILOT = 'openpilot/starpilot/'
OPENDBC = 'opendbc_repo/opendbc/'

# (feature, [(file, regex that must match)], [test paths])
FEATURES = [
  ('Elantra Hybrid 2024 alpha long in classic long AOL',
   [(OPENDBC + 'car/hyundai/classic_long_aol.py', r'ELANTRA_HEV_2024'),
    (OPENDBC + 'safety/modes/hyundai_classic_non_scc_aol.h', r'.')],
   [OPENDBC + 'car/hyundai/tests/test_elantra_hev_long_aol.py']),
  ('AOL re-arms after Park/Reverse and the AOL toggle applies live',
   [(STARPILOT + 'car/hyundai/classic_long_intent.py', r'.'), (STARPILOT + 'car/hyundai/aol.py', r'.')],
   [STARPILOT + 'car/hyundai/tests/test_elantra_gear_rearm.py', STARPILOT + 'car/hyundai/tests/test_elantra_live_aol_toggle.py']),
  ('Elantra Hybrid front radar (bus 1, 0x210-0x21F) and camera+radar tandem leadOne',
   [(OPENDBC + 'car/hyundai/values.py', r'hyundai_radar_210_21f_generated'),
    (OPENDBC + 'car/hyundai/interface.py', r'radar_bus\(ret\) if candidate == CAR\.HYUNDAI_ELANTRA_HEV_2024'),
    (STARPILOT + 'car/hyundai/radar_fusion.py', r'.')],
   [STARPILOT + 'car/hyundai/tests/test_radar_fusion.py']),
  ('Take-a-break (DAW) popup blocked: 8-byte LKAS12 meter held full, 250 ms classic source window',
   [(OPENDBC + 'car/hyundai/daw.py', r'.'), (OPENDBC + 'safety/modes/hyundai_daw.h', r'HYUNDAI_DAW_CLASSIC_SOURCE_MAX_AGE_US 250000U'),
    (OPENDBC + 'safety/modes/hyundai_daw.h', r'HYUNDAI_DAW_METER_FULL')],
   [OPENDBC + 'car/hyundai/tests/test_daw.py', OPENDBC + 'safety/tests/test_hyundai_daw.py']),
  ('Cruise engages at the current speed down to 7 mph',
   [(OP + 'selfdrive/car/cruise.py', r'.')], [OP + 'selfdrive/car/tests/test_cruise_speed.py']),
  ('CEM: stop past a lead stopping at the same spot; release when the stopped lead departs',
   [(STARPILOT + 'conditional_mode/stop_release.py', r'.')],
   [STARPILOT + 'conditional_mode/tests/test_stop.py', STARPILOT + 'conditional_mode/tests/test_stop_release.py']),
  ('Pulse and Glide: steady pulse, zero-torque/EV feather glide, dash-speed band, early phase switches',
   [(STARPILOT + 'controllers/coast.py', r'PULSE_START_LAG_S'), (STARPILOT + 'controllers/coast.py', r'CLUSTER_RATIO_TAU_S')],
   [STARPILOT + 'controllers/tests/test_coast.py']),
  ('Blinker turn guidance: StarPilot turn desires, Turning Left/Right notice',
   [(STARPILOT + 'navigation/intent.py', r'class TurnIntent'), (STARPILOT + 'ui/onroad_lane_alerts.py', r'def turn_intent_alert')],
   [STARPILOT + 'navigation/tests/test_intent.py', STARPILOT + 'navigation/tests/test_signal_guidance.py',
    STARPILOT + 'ui/tests/test_onroad_lane_alerts.py']),
  ('Blinker turns timed to the road (shadow/map/speed), turn beats a queued lane change, green blinker on Turning',
   [(STARPILOT + 'navigation/blinker_turns.py', r"MODES = \('shadow', 'map', 'speed'\)"),
    (STARPILOT + 'navigation/side_road.py', r'def find\('),
    (STARPILOT + 'navigation/intent.py', r'self\.blinker_turn = BlinkerTurn\(\)'),
    (STARPILOT + 'navigation/intent.py', r"log\.Desire\.laneChangeLeft, log\.Desire\.laneChangeRight\)\)"),
    (STARPILOT + 'navigation/navtilesd.py', r'TURN_NEAR_RADIUS_M'),
    (OP + 'selfdrive/modeld/modeld.py', r'"gpsLocationExternal", "gpsLocation", "starpilotCarState"'),
    (OP + 'selfdrive/ui/mici/onroad/alert_renderer.py', r"'turningLeft'"),
    (STARPILOT + 'ui/onroad_alerts.py', r"'turningLeft', 'turningRight'")],
   [STARPILOT + 'navigation/tests/test_blinker_turns.py', STARPILOT + 'navigation/tests/test_side_road.py',
    STARPILOT + 'ui/tests/test_onroad_alerts.py']),
  ('Feature decision logs (turn intent, radar tandem, AOL) and the reserved starpilotLongitudinalPlan struct',
   [(STARPILOT + 'diagnostics/features.py', r"'timed_turning'"), (OP + 'cereal/custom.capnp', r'pulseGlidePhase')], []),
  ('Personal NNLC feedforward with the automatic per-drive trial',
   [(OP + 'selfdrive/controls/lib/latcontrol_torque.py', r"getattr\(self, 'nnlc', None\)"),
    (OP + 'selfdrive/controls/controlsd.py', r'_select_nnlc'), (STARPILOT + 'lateral/nnlc/trial.py', r'def user_override')],
   [STARPILOT + 'lateral/tests/test_nnlc.py']),
  ('Curve guard (opt-in): keep the inside body off a confident inside line or edge',
   [(OP + 'selfdrive/controls/controlsd.py', r'_apply_curve_guard'), (STARPILOT + 'lateral/curve_guard.py', r'class CurveGuard')],
   [STARPILOT + 'lateral/tests/test_curve_guard.py']),
  ('Settings: "L4 custom" tile (personal steering, curve guard, BlueDriver fuel, comma vs you, P&G band)',
   [(OP + 'selfdrive/ui/mici/layouts/settings/settings.py', r'L4 custom'),
    (STARPILOT + 'ui/settings_state.py', r'Destination\.L4_CUSTOM, "L4 Custom"'),
    (STARPILOT + 'ui/runtime_app.py', r'if destination == Destination\.L4_CUSTOM:'),
    (OP + 'selfdrive/ui/mici/layouts/settings/l4_custom.py', r'class L4CustomLayoutMici'),
    (STARPILOT + 'ui/l4_custom.py', r'def fuel_comparison')],
   [STARPILOT + 'ui/tests/test_l4_custom.py', OP + 'selfdrive/ui/tests/test_mici_button_layout.py',
    STARPILOT + 'ui/tests/test_runtime_navigation.py']),
  ('Energy ledger (efficiencyd, ancillary): dash fuel, hybrid power flow, drive mode/tire/temp/map tags, stretch records',
   [(OP + 'system/manager/process_config.py', r'"efficiencyd"'), (OP + 'system/manager/process_health.py', r"'efficiencyd'"),
    (STARPILOT + 'efficiency/efficiencyd.py', r'CarSignals'), (STARPILOT + 'efficiency/car_signals.py', r'TPMS_ADDR = 0x593'),
    (STARPILOT + 'efficiency/ledger.py', r'def record\(self\)')],
   [STARPILOT + 'efficiency/tests']),
  ('Comma-vs-driver fuel comparison and PC trip reports',
   [(STARPILOT + 'efficiency/compare.py', r'def compare\('), (STARPILOT + 'efficiency/pc_summary.py', r'drive_modes_vs_eco'),
    (STARPILOT + 'efficiency/trips.py', r'def drive_mode_comparison')],
   [STARPILOT + 'efficiency/tests/test_compare.py', STARPILOT + 'efficiency/tests/test_trend.py',
    STARPILOT + 'efficiency/tests/test_tags_and_trips.py']),
  ('BlueDriver Pro logger (bluedriverd, opt-in, ancillary): fuel flow, battery, tank, health check',
   [(OP + 'system/manager/process_config.py', r'"bluedriverd"'), (OP + 'system/manager/process_health.py', r"'bluedriverd'"),
    (STARPILOT + 'obd/bluedriver.py', r'def fuel_rate_gps'), (STARPILOT + 'obd/bluedriverd.py', r'async def health_check')],
   [STARPILOT + 'obd/tests']),
  ('Edge (curb) map recorder (edgemapd, ancillary)',
   [(OP + 'system/manager/process_config.py', r'"edgemapd"'), (STARPILOT + 'edgemap/recorder.py', r'class DriveLog')],
   [STARPILOT + 'edgemap/tests']),
  ('Chestnut / model loading timers and diagnostics',
   [(STARPILOT + 'models/runner.py', r'.')],
   [STARPILOT + 'models/tests/test_loader_cpu_affinity.py', STARPILOT + 'models/tests/test_modeld_startup_fallback.py']),
  ('Compact driving controls redraw after background AOL saves',
   [(STARPILOT + 'ui/feature_settings_compact.py', r'.')], [STARPILOT + 'ui/tests/test_compact_aol_save_refresh.py']),
  ('Tailscale starts on boot when installed under /data/tailscale',
   [('launch_chffrplus.sh', r'function tailscale_init'), ('launch_chffrplus.sh', r'^\s+tailscale_init$')], []),
  ('AGNOS 20.5.1: recreate missing /dev/disk/by-partlabel links each boot (Bluetooth radio firmware)',
   [('launch_chffrplus.sh', r'function partition_links'), ('launch_chffrplus.sh', r'^\s+partition_links$')], []),
  ('Drive mode from the 0x2FC announcement + MDPS Sport flag, every CAN batch read for them',
   [('openpilot/starpilot/efficiency/car_signals.py', r'MODE_ADDR = 0x2FC'),
    ('openpilot/starpilot/efficiency/efficiencyd.py', r"sub_sock\('can', conflate=False\)"),
    ('openpilot/starpilot/efficiency/efficiencyd.py', r'SIGNAL_ADDRS = ')],
   ['openpilot/starpilot/efficiency/tests/test_tags_and_trips.py']),
  ('NNLC drive choice re-logged about once per segment',
   [('openpilot/selfdrive/controls/controlsd.py', r'NNLC_RELOG_S = '),
    ('openpilot/selfdrive/controls/controlsd.py', r"getattr\(self, '_nnlc_event', None\)")],
   ['openpilot/selfdrive/controls/tests/test_nnlc_relog.py']),
  ('Battery % glitch filter in the fuel ledger',
   [('openpilot/starpilot/efficiency/ledger.py', r'def _smoothed_soc'), ('openpilot/starpilot/efficiency/ledger.py', r'SOC_CONFIRM = ')],
   ['openpilot/starpilot/efficiency/tests/test_measured_fuel.py']),
  ('Map curve speed in shadow mode (map curves, torque-limited targets, curve memory learning from takeovers)',
   [('openpilot/starpilot/curve_speed/curvemapd.py', r'class Shadow'),
    ('openpilot/starpilot/curve_speed/curve_memory.py', r'def verdict_for'),
    ('openpilot/starpilot/navigation/side_road.py', r'include_motorway'),
    ('openpilot/starpilot/navigation/side_road.py', r'def _take_branch'),
    ('openpilot/starpilot/curve_speed/camera.py', r'def which_branch'),
    ('openpilot/starpilot/curve_speed/curvemapd.py', r'class CameraCheck'),
    ('openpilot/system/manager/process_config.py', r'"curvemapd"'),
    ('openpilot/system/manager/process_health.py', r"'curvemapd'")],
   ['openpilot/starpilot/curve_speed/tests/test_map_curve_speed.py']),
  ('Galaxy remote tunnel restarts on a network change and uses a 10 s keepalive',
   [('openpilot/starpilot/galaxy/remote_service.py', r'def network_route'),
    ('openpilot/starpilot/galaxy/remote_service.py', r'moved = current_route is not None'),
    ('openpilot/starpilot/galaxy/remote.py', r'tcpMuxKeepaliveInterval = 10')],
   ['openpilot/starpilot/galaxy/tests/test_remote_service.py']),
]


def check_anchors() -> list[str]:
  missing = []
  for name, anchors, _ in FEATURES:
    for path, pattern in anchors:
      file = ROOT / path
      try:
        text = file.read_text(errors='replace')
      except OSError:
        missing.append(f'{name}: {path} is missing')
        continue
      if not re.search(pattern, text, re.MULTILINE):
        missing.append(f'{name}: {path} no longer contains /{pattern}/')
  return missing


def run_tests() -> list[str]:
  failed = []
  for name, _, tests in FEATURES:
    if not tests:
      continue
    paths = [t for t in tests if (ROOT / t).exists()]
    gone = sorted(set(tests) - set(paths))
    if gone:
      failed.append(f'{name}: test files missing: {", ".join(gone)}')
    if not paths:
      continue
    cmd = ['toolbox', 'run', '-c', 'op-ubuntu', 'bash', '-lc', f'cd {ROOT} && ./dev pytest -q -p no:cacheprovider ' + ' '.join(paths)]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    summary = next((line for line in reversed(result.stdout.splitlines()) if ' passed' in line or ' failed' in line), 'no summary')
    print(f'  {"PASS" if result.returncode == 0 else "FAIL"}  {name}: {summary.strip()}')
    if result.returncode != 0:
      failed.append(f'{name}: {summary.strip()}')
  return failed


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument('--tests', action='store_true', help="also run each feature's tests")
  args = parser.parse_args()
  missing = check_anchors()
  print(f'{len(FEATURES)} features, anchors: {"all present" if not missing else f"{len(missing)} missing"}')
  for line in missing:
    print('  MISSING ', line)
  failed = run_tests() if args.tests else []
  return 1 if missing or failed else 0


if __name__ == '__main__':
  sys.exit(main())
