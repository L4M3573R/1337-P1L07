#!/usr/bin/env python3
"""Verify every 1337-P1L07 customization is still present and working after an upstream (StarPilot Dom) merge.

  .venv/bin/python scripts/check_1337.py            # anchors only: each feature's key code is still there
  .venv/bin/python scripts/check_1337.py --tests    # anchors, then every feature's tests in one ./dev pytest run

This is the fork's standard test run: the Elantra, its panda safety mode and everything built here. Tests for other
makes are not part of it (2026-11-18: the user wants a clean run to mean "what we use works").

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
    (STARPILOT + 'navigation/intent.py', r"log\.Desire\.laneChangeLeft if cs\.leftBlinker else"),
    (STARPILOT + 'navigation/navtilesd.py', r'env\.map_enabled or env\.turns_enabled'),
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
  ('Turns: no push back against the driver mid-turn (turn yield itself was removed, 092000f5)',
   [(OP + 'selfdrive/controls/controlsd.py', r'from openpilot\.starpilot\.lateral\.turn_yield import no_push_back'),
    (STARPILOT + 'lateral/turn_yield.py', r'def no_push_back')],
   [STARPILOT + 'lateral/tests/test_turn_yield.py']),
  ('No lane change into a lane that is not there (road edge close, no line beyond)',
   [(STARPILOT + 'lateral/adjacent_lane_evidence.py', r'def lane_clearly_absent'),
    (OP + 'selfdrive/controls/lib/desire_helper.py', r'not no_lane and'),
    (OP + 'selfdrive/modeld/modeld.py', r'no_lane = direction != 0 and lane_clearly_absent\(modelv2_send\.modelV2, direction\)')],
   [STARPILOT + 'lateral/tests/test_auto_lane_change.py', OP + 'selfdrive/controls/tests/test_desire_helper_preferences.py']),
  ('Settings: "L4 custom" tile (NNLC, curve guard, BlueDriver fuel, comma vs you, P&G band)',
   [(OP + 'selfdrive/ui/mici/layouts/settings/settings.py', r'L4 custom'),
    (STARPILOT + 'ui/settings_state.py', r'Destination\.L4_CUSTOM, "L4 Custom"'),
    (STARPILOT + 'ui/runtime_compact.py', r'if destination == runtime\.Destination\.L4_CUSTOM:'),  # moved there by upstream 2026-11-14
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
  ('Self-learning fuel efficiency (shadow, PC summary): EV cut-off, Pulse and Glide bands in the time budget, downhill carry',
   [(STARPILOT + 'efficiency/learner.py', r'TIME_BUDGET_MIN_PER_HR = 4\.0'), (STARPILOT + 'efficiency/pc_summary.py', r'learner\.learn_stored')],
   [STARPILOT + 'efficiency/tests/test_learner.py']),
  ('1337 tracing: traced daily trace file, lead-loss / Pulse and Glide / slow-lookup decision logs, PC timeline',
   [(OP + 'system/manager/process_config.py', r'"traced"'), (OP + 'system/manager/process_health.py', r"'traced'"),
    (STARPILOT + 'diagnostics/features.py', r'class LeadLossLog'), (OP + 'selfdrive/controls/radard.py', r'LeadLossLog\(\)'),
    (STARPILOT + 'controllers/coast.py', r"ChangeLog\('pulse_glide'\)"), (STARPILOT + 'diagnostics/trace_report.py', r'class Timeline')],
   [STARPILOT + 'tests/test_trace.py']),
  ('Front radar trimmed decoding + REL_ACCEL on points, radar shadow logs, car yaw rate',
   [('opendbc_repo/opendbc/car/hyundai/radar_interface.py', r'self\.relative_accel = '),
    ('opendbc_repo/opendbc/car/hyundai/radar_interface.py', r"'REL_ACCEL'\)"),
    ('opendbc_repo/opendbc/car/hyundai/carstate.py', r'ret\.yawRate = -cp\.vl\["ESP12"\]\["YAW_RATE"\]'),
    (STARPILOT + 'diagnostics/features.py', r'class LeadAccelShadow'), (OP + 'selfdrive/controls/radard.py', r'LeadAccelShadow\(\)'),
    (STARPILOT + 'curve_speed/curvemapd.py', r'def measured_curvature')],
   [STARPILOT + 'tests/test_trace.py', STARPILOT + 'curve_speed/tests/test_map_curve_speed.py']),
  ('Trace coverage: program crashes/restarts, error lines, Force Stop / curve / speed-sign / efficiency / edge map / FLM logs',
   [(STARPILOT + 'diagnostics/traced.py', r'def processes'), (STARPILOT + 'diagnostics/traced.py', r"sub_sock\('errorLogMessage'"),
    (STARPILOT + 'diagnostics/features.py', r'def log_error'), (OP + 'selfdrive/controls/plannerd.py', r"ChangeLog\('force_stop'"),
    (OP + 'selfdrive/controls/plannerd.py', r"ChangeLog\('curve_speed'"), (STARPILOT + 'longitudinal/force_stop.py', r'def _decision'),
    (STARPILOT + 'speed_limits/vision/producer.py', r"ChangeLog\('speed_sign'"), (STARPILOT + 'efficiency/efficiencyd.py', r"ChangeLog\('efficiency_sources'"),
    (STARPILOT + 'edgemap/edgemapd.py', r"feature='edge_map'"), (STARPILOT + 'flm/offroad_daemon.py', r"ChangeLog\('steer_learner'\)")],
   [STARPILOT + 'tests/test_trace_coverage.py']),
  ('Self-learning fuel experiments (experimenter, plannerd host, planner hooks, tagged ledger, learner)',
   [(STARPILOT + 'efficiency/experiment.py', r'class Experimenter'), (STARPILOT + 'efficiency/experiment_host.py', r'class ExperimentHost'),
    (OP + 'selfdrive/controls/plannerd.py', r'ExperimentHost\('), (OP + 'selfdrive/controls/lib/longitudinal_planner.py', r'efficiency_plan=None'),
    (STARPILOT + 'efficiency/ledger.py', r"'exp': self\.experiment"), (STARPILOT + 'efficiency/experiment_learn.py', r'def judge'),
    (STARPILOT + 'efficiency/efficiencyd.py', r'def start_judge')],   # the judge runs beside efficiencyd as a drive starts
   [STARPILOT + 'efficiency/tests/test_experiment.py', STARPILOT + 'efficiency/tests/test_experiment_learn.py',
    STARPILOT + 'efficiency/tests/test_experiment_host.py', STARPILOT + 'efficiency/tests/test_experiment_planner.py']),
  ('Turn speed for signalled turns (learned per side and sharpness, eased into by plannerd)',
   [(STARPILOT + 'navigation/turn_speed.py', r'class TurnSpeedHost'), (STARPILOT + 'navigation/intent.py', r'turn_speed import write_status'),
    (OP + 'selfdrive/controls/plannerd.py', r'TurnSpeedHost\('),
    (OP + 'selfdrive/controls/lib/longitudinal_planner.py', r'turn_ceiling_mps: float \| None = None'),
    (STARPILOT + 'navigation/intent.py', r'def _approach\(self, sm, now_ns: int, cs, current_desire\)'),
    (OP + 'selfdrive/modeld/modeld.py', r'navigation_turn\.no_lane_side = direction if no_lane else 0')],
   [STARPILOT + 'navigation/tests/test_turn_speed.py']),
  ('Curve memory ignores signalled turns; blinker turns trigger 3.2 s + 7 m early; Pulse and Glide badge',
   [(STARPILOT + 'curve_speed/curve_memory.py', r"verdict = 'turn' if signalled"), (STARPILOT + 'navigation/blinker_turns.py', r'TRIGGER_S = 3\.2'),
    (STARPILOT + 'ui/onroad.py', r'def _pulse_glide_badge'), (STARPILOT + 'diagnostics/features.py', r'LEAD_RADAR_TRIGGER = -1\.2')],
   [STARPILOT + 'curve_speed/tests/test_map_curve_speed.py', STARPILOT + 'navigation/tests/test_blinker_turns.py',
    STARPILOT + 'ui/tests/test_pulse_glide_badge.py']),
  ('BlueDriver direct fuel rate (019D) and hybrid battery voltage (019A)',
   [(STARPILOT + 'obd/bluedriver.py', r"'019D': lambda"), (STARPILOT + 'obd/bluedriverd.py', r'fuel_direct')],
   [STARPILOT + 'obd/tests/test_bluedriver.py']),
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
  ('Road search vectorized and moved out of modeld into sideroadd',
   [('openpilot/starpilot/navigation/side_road.py', r'def _project_many'),
    ('openpilot/starpilot/navigation/intent.py', r'MapTurnFinder\(remote=True\)'),
    ('openpilot/starpilot/navigation/sideroadd.py', r'def step'),
    ('openpilot/system/manager/process_config.py', r'"sideroadd"')],
   ['openpilot/starpilot/navigation/tests/test_sideroadd.py']),
  ('DR4L upload progress line on the home screen',
   [('openpilot/starpilot/ui/dr4l_upload.py', r'class UploadStatus'),
    ('openpilot/starpilot/ui/home.py', r'state.upload'),
    ('openpilot/starpilot/ui/runtime_snapshot.py', r'_upload_status')],
   ['openpilot/starpilot/ui/tests/test_dr4l_upload.py']),
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
  ('Onroad screen paced to the camera; onroad animations step by real frame time',
   [(OP + 'system/ui/lib/application.py', r'def add_frame_pacer'), (OP + 'system/ui/lib/frame_filter.py', r'class FrameFilter'),
    (OP + 'selfdrive/ui/onroad/cameraview.py', r'def _pace_to_camera'), (OP + 'selfdrive/ui/mici/onroad/model_renderer.py', r'FrameFilter\(')],
   [OP + 'system/ui/tests/test_frame_filter.py', OP + 'system/ui/tests/test_frame_timing.py', STARPILOT + 'ui/tests/test_camera_resources.py']),
  ('Fuel learner limits: veto as a bound, budget carried across drives, pitch zero, 5-drive win test',
   [(STARPILOT + 'efficiency/dials.py', r'def blocked_by_veto'), (STARPILOT + 'efficiency/experiment.py', r"BUDGET_FILE = 'budget\.json'"),
    (STARPILOT + 'efficiency/experiment_learn.py', r'DIAL_MIN_DRIVES = 5'), (STARPILOT + 'efficiency/experiment_learn.py', r'def pitch_zero')],
   [STARPILOT + 'efficiency/tests']),
  ('Fuel learner: cruise and hill speed picked from the measured MPG by speed, per grade and condition',
   [(STARPILOT + 'efficiency/speed_curve.py', r'def condition_tables'), (STARPILOT + 'efficiency/experiment.py', r'SPEED_PICKS = '),
    (STARPILOT + 'efficiency/experiment_host.py', r'def cruise_over_mph'), (STARPILOT + 'efficiency/experiment_learn.py', r'speed_curve\.condition_tables')],
   [STARPILOT + 'efficiency/tests/test_speed_curve.py', STARPILOT + 'efficiency/tests/test_cruise_speed.py']),
  ('BlueDriver logs the engine computer vehicle speed',
   [(STARPILOT + 'obd/bluedriver.py', r"\('010D', 2\)")], [STARPILOT + 'obd/tests']),
  ('Elantra Hybrid EPS limits 400 / 10 / 9 (car code and panda, selected by the unchanged safety word)',
   [(OPENDBC + 'safety/modes/hyundai.h', r'HYUNDAI_STEERING_LIMITS_ELANTRA_HEV_2024 = HYUNDAI_LIMITS_RT\(400, 10, 9, 375\)'),
    (OPENDBC + 'car/hyundai/values.py', r'def elantra_hev_2024_eps')],
   [OPENDBC + 'car/hyundai/tests/test_elantra_hev_eps_limits.py']),
  ("The Elantra's panda safety mode (classic Hyundai) and the panda protocol",
   [(OPENDBC + 'safety/modes/hyundai.h', r'hyundai_tx_hook'), (OPENDBC + 'safety/tests/conftest.py', r'RELEASE_ONLY')],
   [OPENDBC + 'safety/tests/test_hyundai.py', OPENDBC + 'safety/tests/test_hyundai_classic_long_aol.py',
    OPENDBC + 'safety/tests/test_hyundai_main_rearm.py', OPENDBC + 'safety/tests/test_hyundai_classic_cancel_joined.py',
    OPENDBC + 'safety/tests/test_hyundai_blended_cruise_ack.py', OPENDBC + 'safety/tests/misra/test_mutation.py',
    OP + 'selfdrive/pandad/tests']),
  ('Stop-line finder (shadow) and Force Stop lead identity',
   [(OP + 'system/manager/process_config.py', r'"stop_lined"'), (STARPILOT + 'conditional_mode/stop.py', r'OWN_LEAD_MAX_JUMP_M')],
   [STARPILOT + 'stop_line/tests', STARPILOT + 'conditional_mode/tests/test_stop_plan.py']),
  ('Test runner: ./dev pytest runs without the desktop-app switches (167 false failures, 2026-11-18)',
   [('tools/host_runtime.py', r"PYTEST_UNSET_ENV = \('SIMULATION', 'SKIP_FW_QUERY', 'NOBOARD'\)")],
   ['tools/ci/tests/test_dev_runtime.py']),
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
  """Every feature's tests in one ./dev pytest run (each ./dev start costs a build check), then per feature: PASS
  unless one of its files had a failure or error. Inside the Claude Desktop distrobox the run goes to the host, where
  the compiler is."""
  import os
  import shlex
  failed, wanted = [], {}
  for name, _, tests in FEATURES:
    paths = [t for t in tests if (ROOT / t).exists()]
    gone = sorted(set(tests) - set(paths))
    if gone:
      failed.append(f'{name}: test files missing: {", ".join(gone)}')
    wanted[name] = paths
  paths = sorted({p for ps in wanted.values() for p in ps})
  inner = (f'cd {shlex.quote(str(ROOT))} && SCONSFLAGS="--ccflags=-Wno-error" ./dev pytest -q -rfE -p no:cacheprovider '
           + ' '.join(map(shlex.quote, paths)))
  cmd = (['distrobox-host-exec'] if os.environ.get('CONTAINER_ID') else []) + ['bash', '-c', inner]
  result = subprocess.run(cmd, capture_output=True, text=True, errors='replace', check=False)
  lines = result.stdout.splitlines()
  summary = next((line.strip(' =') for line in reversed(lines) if ' passed' in line or ' failed' in line or ' error' in line), 'no summary')
  broken = sorted({line.split()[1].split('::')[0] for line in lines if line.startswith(('FAILED ', 'ERROR ')) and len(line.split()) > 1})
  print(f'tests: {summary}')
  if result.returncode != 0 and not broken:
    failed.append(f'the test run itself failed (exit {result.returncode}): {summary}')
  for name, feature_paths in wanted.items():
    if not feature_paths:
      continue
    bad = [b for b in broken if any(b == p or b.startswith(p.rstrip('/') + '/') for p in feature_paths)]
    print(f'  {"FAIL" if bad else "PASS"}  {name}' + (f': {", ".join(bad)}' if bad else ''))
    if bad:
      failed.append(f'{name}: {", ".join(bad)}')
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
  if args.tests:
    print('ALL CLEAR' if not (missing or failed) else f'{len(missing)} anchors missing, {len(failed)} features failing')
  return 1 if missing or failed else 0


if __name__ == '__main__':
  sys.exit(main())
