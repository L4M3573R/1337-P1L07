# 1337-P1L07 changes on top of StarPilot

Everything this fork adds or changes relative to upstream StarPilot (`upstream` = StarPilotHQ/StarPilot, branch
`Dom`). Use it on every upstream merge: compare, keep both sides, rebuild what is generated, verify, never lose a
feature. `scripts/check_1337.py` checks the same list automatically.

## Merge checklist

1. `git fetch upstream Dom`, then `git branch -f backup/1337-8374-before-dom-merge-<date> HEAD`.
2. Commit or set aside any work in progress (a WIP commit, never a bare `git stash`).
3. `git merge --no-ff --no-commit upstream/Dom`.
4. Source conflicts: keep **both** behaviors. Read the feature below that owns the file before choosing.
5. Generated files are never hand-merged. Take either side as a placeholder, then rebuild:
   - **Panda firmware** (`panda/board/obj/*.bin.signed`, `gitversion.h`, `version`): rebuild whenever either side
     changed `opendbc_repo/opendbc/safety/` or `panda/board/`:
     `T="$(git ls-files 'panda/board/obj/*.bin.signed' | tr '\n' ' ') panda/board/obj/version";
     COMMA_DOCKER_RUN_USER=0:0 scripts/laptop_device_build.sh scons --no-scrub $T` (takes a while).
   - **pandad / libpanda** (`openpilot/selfdrive/pandad/pandad`, `libpanda.a`): rebuild the same way when either
     side changed their C++ sources (this fork changed the AOL transport profile in libpanda).
   - **cereal generated C++** (`openpilot/cereal/gen/cpp/custom.capnp.*`): regenerate if `custom.capnp` changed on
     both sides.
6. Commit the merge, then `.venv/bin/python openpilot/common/prebuilt_manifest.py .`, commit `prebuilt.json`, and
   verify the receipt (`valid_receipt(...)`) before pushing.
7. `.venv/bin/python scripts/check_1337.py --tests`: every anchor present, every feature's tests passing.
8. Note an AGNOS bump in `launch_env.sh` (`AGNOS_VERSION`): the comma updates its OS on install (extra reboots).
9. Push, then install on the comma (force Offroad, `kill -HUP` updated, wait for staging, reboot).

### Known test failures that are not merge damage

These fail the same way on pristine upstream under the `./dev` wrapper (they read the PC's stored settings or
need native libraries) and pass when run directly with the venv:
- `starpilot/lateral/tests/test_lane_runtime.py::...test_body_guard_rejects_incorrect_delay_rate...`
- `opendbc/car/hyundai/tests/test_*discovery*.py` (fingerprint discovery under the wrapper)
- `starpilot/controllers/tests/test_wheel_media_audit.py` (reads stored button mappings)
- the native font/UI render tests (about 24)
- `selfdrive/ui/tests/test_camera_speed_gates.py::test_connection_retries_advance_when_recorded_clock_is_frozen_or_rewound`
  (2), `starpilot/ui/tests/test_laboratory_home.py`, `test_settings_actions.py` compact targets, and
  `starpilot/replay/tests/test_export.py` (needs `libmedia_mux.so`) under `./dev` (2026-11-15, failing on HEAD too)
- `starpilot/models/tests/test_modeld_startup_fallback.py::test_jetlink_gets_the_frames_delay_before_every_run`: upstream
  added `+ volkswagen_tau` to modeld's `lat_delay` line without updating this AST test (fails on pristine Dom, 2026-11-16)
- `selfdrive/controls/tests/test_conditional_runtime_loop.py::...test_automatic_chill_ignores_retired_slc_modes_but_keeps_native_radar_veto`
  ('no_trigger' != 'ccm_speed'): fails on pristine Dom too (2026-11-16)
- `scripts/check_1337.py --tests` runs through `toolbox run -c op-ubuntu`, which only the laptop has; on the PC run the
  feature tests with `./dev pytest` directly (2026-11-16)

### Schema notes
- `StarPilotLongitudinalPlan` (`custom.capnp`): upstream took `@7`/`@8` for `departureAlertKind`/`departureAlertMonoTime`,
  so the 2026-11-16 merge moved the 1337 Pulse and Glide and CEM stop fields from `@7`-`@20` to `@9`-`@22` (Cap'n Proto
  allows no gaps). Code reads them by name; logs recorded before that merge decode those fields wrong.

## Features

Each entry: what it does, where it lives, the commits, and what must survive a merge.

### Car support (Elantra Hybrid 2024, HYUNDAI_ELANTRA_HEV_2024)
- **Alpha long in classic long AOL** (`c94919795b`, firmware `21aa8bc700`, `b289dcae88`):
  `opendbc/car/hyundai/classic_long_aol.py`, `long_buttons.py`, `safety/modes/hyundai.h`,
  `hyundai_classic_non_scc_aol.h`, `starpilot/car/hyundai/aol_policy.h`; pandad/libpanda rebuilt.
- **AOL survives Park/Reverse and re-arms; toggle applies live** (`9d82b84d8a`, `fb1026580a`, `f912e790b9`):
  `starpilot/car/hyundai/classic_long_intent.py`, `aol.py`, `selfdrive/car/card.py`, `dmonitoringd.py`.
- **Front radar on bus 1 (0x210-0x21F) and camera+radar tandem leadOne** (`38b72f01da`, `3d9d2531a7`):
  `opendbc/car/hyundai/values.py` (radar DBC), `interface.py` (radar bus for this car), `radar_interface.py`,
  `starpilot/car/hyundai/radar_fusion.py`, `selfdrive/controls/radard.py`.
  Watch upstream radar changes (e.g. the lateral sign fix) against the fusion code.
- **Take-a-break popup blocked** (`1ec07d8d02`, `f70946c34e`): `opendbc/car/hyundai/daw.py` (8-byte LKAS12 meter
  held full), `safety/modes/hyundai_daw.h` (`HYUNDAI_DAW_CLASSIC_SOURCE_MAX_AGE_US 250000U`). Firmware rebuild.
- **Cruise engages down to 7 mph** (`810b0428c5`): `selfdrive/car/cruise.py`.

### Longitudinal
- **Pulse and Glide** (`ba0c53b85a`, `7c751f99ca`, `4f1ebefe17`, `fe95dc1425`): `starpilot/controllers/coast.py`
  (steady pulse, zero-torque/EV feather glide below ~63 mph, dash-speed band, `PULSE_START_LAG_S`,
  `ClusterRatioFilter`), `selfdrive/controls/lib/longitudinal_planner.py`. Survives stops: the filtered vEgo dips a
  hair below zero at standstill (`STANDSTILL_NOISE_MPS`), which used to wipe the driver's Pulse and Glide choice.
  The glide never drops more than 5 mph under the set speed (2026-11-15): the L4 band is 3 or 5 mph (5 or 8 km/h,
  `PG_DELTAS_*`, `set_pg_delta` rejects more) and `coast.py` clamps any stored band to `MAX_BAND_MPS`.
- **CEM stops** (`dac4e3ce32`, `1102863c55`): `starpilot/conditional_mode/stop_release.py`, `stop.py`.
- **Stop-line finder, shadow only** (2026-11-15): `starpilot/stop_line/`. A small model of our own (MobileNetV3-small,
  one narrow-road frame in; stop-ahead probability, distance up to 120 m and its uncertainty out) that says where to
  stop at a light or sign even when the line is worn away, next to the driving model. `stop_lined` (onroad,
  ancillary, little cores, nice 15) runs `<storage>/stop_line/stop_line.onnx` through cv2.dnn about 4 Hz (never over
  40% of a core) if that file is there, fuses frames into one stop point along the odometer (`fusion.py`) and picks
  ours when sure, else chestnut's planned stop (where modelV2's planned speed reaches zero, `labels.model_stop_distance`).
  Nothing reads it yet: it logs `starpilot_feature` / `stop_line` choices and a `what=rest` event per stop scoring ours
  and chestnut at 60/30/15 m against where the car stopped. Trained on the PC (see "Outside the repo").

### Lateral
- **Personal NNLC with automatic trial** (`6c339a40be`, `c5e36c122d`): `starpilot/lateral/nnlc/`,
  `latcontrol_torque.py` (NNLC hook after friction), `controlsd.py` (`_select_nnlc`; FLM off on NNLC drives).
  Trained off-car by the DR4L container (see "Outside the repo").
- **Curve guard (opt-in)** (`9110e41e79`): `starpilot/lateral/curve_guard.py`, `controlsd.py` (`_apply_curve_guard`). Since
  2026-11-16 a blinker alone no longer stands it down (signalling for a merge kept it off on an on-ramp curve); only a
  lane change under way does.
- **No lane change into a lane that is not there** (2026-11-16): `adjacent_lane_evidence.lane_clearly_absent` (far line
  under 15 %, road edge on that side under 1 m std and within 2.5 m of the lane line over 30 m), `desire_helper.py`
  (`no_lane`: neither a nudge nor auto starts the change), `modeld.py`. Replayed on the 36 lane changes of 2026-11-14/15:
  blocks 9, all turns signalled above 20 mph or the on-ramp cut, none of the 27 real lane changes.
- **Early turn slowdown** (2026-11-16): `intent._approach`. Signalled at 20-45 mph (`APPROACH_MAX_MPH`), the mapped road
  ahead goes to turn speed early enough to ease down to the learned speed (36 mph to 9 mph needs ~120 m), only when the
  blinker can only mean a turn: no lane that side (modeld passes `lane_clearly_absent` as `no_lane_side`) or a lane change
  toward it just finished on this blinker (`BlinkerTurn.restart_signal`). Never during a lane change; map answers older
  than 3 s are ignored; the 150 m signal limit applies. Logs `approach_reason`.
- **Parking-lot EPS test switch** (2026-11-16): `hyundai/carcontroller.eps_test_max_angle`: `/data/starpilot/eps_test.json`
  {"max_angle": deg} moves the >90 degree cut-out (stock 85 deg after 89 frames) for that drive. Test only; remove the
  file after the test. The test pattern itself is a script run over SSH with JoystickDebugMode, not in the repo.
- **Turn yield** (2026-11-16): `starpilot/lateral/turn_yield.py`, `controlsd._turn_yielding`. Below 20 mph with one
  blinker, while the driver steers toward it, the lateral controller runs inactive (no torque, state reset; latActive
  stays true) and takes back over 0.3 s after release; logs `turn_yield` changes. The controller had pushed back at full
  torque in 7 of 37 comma-steered turns; a replay covers 88 % of those samples.

### Turns
- **Blinker turn guidance** (`36a3dc1aa9`, `d2ecbfb5c2`, `42de700f0b`, `4f1ebefe17`): `starpilot/navigation/intent.py`,
  `starpilot/ui/onroad_lane_alerts.py` (Turning Left/Right).
- **Turns timed to the road** (`2792a5f492`): `navigation/side_road.py`, `navigation/blinker_turns.py`
  (shadow/map/speed; default shadow), `intent.py` (turn beats a queued lane change below 20 mph), `navtilesd.py`
  (nearby tiles), `modeld.py` (GPS subscriptions), green blinker on Turning in `alert_renderer.py` and
  `starpilot/ui/onroad_alerts.py`.
- **One blinker, one turn** (2026-11-16): `blinker_turns.BlinkerTurn`. After a turn is done, or the car has turned
  without the comma, the blinker takes no more mapped roads until it goes off; a mapped road more than
  `SIGNAL_AHEAD_M` (150 m) from where the blinker came on is not the one signalled for (queue creep behind a car not
  counted); a nudge toward the blinker starts the turn even with a mapped road still ahead (driveways Mapbox lacks).
  Before sideroadd's first answer the reason is 'map: looking up the road', not 'no map here'. (The no-blinker straight
  guard, shadow only, was removed the same day: a better driving model is the planned fix for turning without a blinker.)

### Efficiency and fuel
- **Energy ledger** (`246ace1369`, `9eafb2765a`, `eb7c0ce812`, `9355d3146c`, `575dc06820`, `835eb17fa5`):
  `starpilot/efficiency/` (efficiencyd ancillary in `process_config.py` and `process_health.py`), dash fuel,
  hybrid power flow (0x595), drive mode / tire pressure / map limit tags (`car_signals.py`), per-stretch records.
  Drive mode comes from the car's 0x2FC byte 7 announcement (0x40 Eco, 0x60 Sport, 0xA0 Smart) plus the MDPS11
  Sport flag, never 0x44E; efficiencyd reads every CAN batch (not conflated) for those two addresses so the few-frame
  announcement is not missed.
- **BlueDriver Pro logger** (`eb47adb2ca`): `starpilot/obd/` (bluedriverd, opt-in, ancillary), measured fuel and
  battery in the ledger, tank level, read-only health check.
- **Comma vs driver and PC reports** (`575dc06820`, `835eb17fa5`): `efficiency/compare.py`, `trend.py`, `trips.py`,
  `pc_summary.py`.
- **Self-learning fuel efficiency (shadow)**: `efficiency/learner.py`, shown by `pc_summary.py` (`learner`). From
  BlueDriver measured, battery-adjusted fuel only: learns the EV cut-off speed for feather glides (fixed 63 mph in
  `coast.py`), relearned every run from the last 14 days (`EV_WINDOW_DAYS`, it moves with the weather) by the BlueDriver's
  engine rpm (ledger flow entries carry `FLOW_LEN` 7: BlueDriver readings and engine-off counts, from `efficiencyd`
  `obd_engine`), falling back to 0x595 power flow; where Pulse and Glide beats steady cruise per speed band and the deepest band the driver's time budget
  allows (`TIME_BUDGET_MIN_PER_HR` 4, the driver's 4 min/hr, at most 5 mph under), and what running up to 3 mph over the set speed on open
  downhills would keep. Reports only; nothing reads it for control. Tests in `efficiency/tests/test_learner.py`.

### Settings and UI
- **L4 custom tile** (`c5e36c122d`, `e825ae80af`, `b91e0d2fa9`, `eb47adb2ca`, `575dc06820`):
  `selfdrive/ui/mici/layouts/settings/settings.py`, `.../l4_custom.py`, `starpilot/ui/l4_custom.py`. The comma shows
  StarPilot's compact menu, so the card must also be in `starpilot/ui/settings_state.py` (`Destination.L4_CUSTOM`,
  first in `COMPACT_MENU`) and opened in `starpilot/ui/runtime_app.py` (`_open_compact_destination`).
- **Compact driving controls redraw after AOL saves** (`92fdf51393`).
- **Onroad screen paced to the camera, animations in real time** (2026-11-15): `system/ui/lib/application.py`
  (`frame_dt`, `add_frame_pacer`, `expected_fps` for the FPS monitor), `system/ui/lib/frame_filter.py`
  (`FrameFilter`, `FrameBounceFilter`), `selfdrive/ui/onroad/cameraview.py` (`_pace_to_camera`; EGL image re-attached
  only when it changes, `egl.bind_egl_image_to_texture(attach=)`). The onroad filters in `selfdrive/ui/mici/onroad/`,
  `selfdrive/ui/onroad/model_renderer.py`, `model_source.py`, `ui_state.py` (brightness) and `starpilot/ui/onroad_*.py`
  use the frame filters; menus keep `FirstOrderFilter`. Measured on the comma 4: 52% -> 41% of a core, steady 20 FPS.

### Other
- **Edge (curb) map recorder** (`fb1026580a`): `starpilot/edgemap/`, edgemapd ancillary.
- **Chestnut/model loading** (`07cbcf0f22`, `dab4cd3c35`, `abc4eddd03`): `starpilot/models/runner.py`,
  `runtime_bridge/runtime.py`, `modeld.py`.
- **Decision logs** (`4f1ebefe17`, `2792a5f492`): `starpilot/diagnostics/features.py`, reserved
  `starpilotLongitudinalPlan` in `cereal/custom.capnp`.
- **1337 tracing** (2026-11-13): `starpilot/diagnostics/traced.py` (traced, onroad ancillary, little cores, nice 19)
  writes `<storage>/trace/YYYY-MM-DD.jsonl` on the comma: every `starpilot_feature` decision event, an `alert` CPU
  snapshot (cores, temperature, busiest processes) on any disengaging or lag/fault warning, and a per-minute `load` line;
  keeps 14 days. New decision logs: `lead_loss` (`features.LeadLossLog`, fed by `radard.py`), `pulse_glide` on/off with
  the reason (`coast.py` `change_reason` / `admission_failure`), slow map lookups (`features.log_if_slow` in
  `sideroadd.py` and `curvemapd.py`). PC timeline: `python -m openpilot.starpilot.diagnostics.trace_report <folder>
  <route> [--from M:SS --to M:SS]`. Tests in `starpilot/tests/test_trace.py`.
- **Front radar: trimmed decoding + REL_ACCEL** (2026-11-14): `opendbc/car/hyundai/radar_interface.py` decodes only the
  used fields of the 0x210-0x21F / MRR30 banks (STATE, LONG_DIST, LAT_DIST, REL_SPEED, REL_ACCEL; identical points on
  15519 recorded radar updates, ~half the parse time in card) and, for the Elantra Hybrid (`relative_accel`), puts each
  track's REL_ACCEL in the point's reserved `deprecated.aRel` with `deprecated.measured=True`. Tracks stay keyed by
  slot: OBJECT_ID repeats within a cycle (not unique). REL_ACCEL showed a lead's braking ~0.5 s before radard's Kalman
  aLeadK on 63 onsets over 70 min of drives.
- **Radar shadow logs** (2026-11-14): `features.LeadAccelShadow` (`lead_accel_shadow` events: how much earlier REL_ACCEL
  showed each lead braking onset) and `LeadLossLog(track_alive=...)` (`radar_still_tracking`, `same_radar_track`: the
  lead-hold shadow), fed in `radard.py`. Log only.
- **Car yaw rate** (2026-11-14): `opendbc/car/hyundai/carstate.py` sets `carState.yawRate` from ESP12 YAW_RATE (deg/s,
  right positive) for the Elantra Hybrid only (nothing on Hyundai read it; 0.994 correlation with curvature x speed);
  `curvemapd.measured_curvature` uses yaw/speed above 5 m/s, else controlsState.curvature.
- **BlueDriver direct fuel rate + battery voltage** (2026-11-14): `starpilot/obd/bluedriver.py` PIDs 019D (the ECU's
  fuel rate, matched the MAF/lambda estimate at idle) and 019A (hybrid battery system voltage; current needs a
  multi-frame read); `bluedriverd.py` integrates `fuel_direct_g` alongside `fuel_g` for comparison.
- **Trace coverage: crashes, errors, every system** (2026-11-14): `traced.py` also writes `process` lines (a program
  that died while it should run, with its exit code / killing signal and a CPU snapshot; restarted under a new pid;
  still down after 5 s; back up) from managerState, and `error` lines (every error-level app-log line, with the
  traceback tail, once per program and message per 30 s) from errorLogMessage; `trace_report` shows both from rlogs.
  `features.log_error` / `log_event` replace silent `except: pass` in the shadow and daemon code (rate-limited, error
  level). `ChangeLog(min_gap_s=...)` folds a flickering decision into one event with `flickers`. New decision logs:
  `force_stop` (plannerd: the gate, what the stop follows - light / sign / queue / holding - or what holds it back:
  lead, curve, turn, override, light too far, sign too fast; `ForceStop.decision`, `ForceStopRuntime.gate`),
  `curve_speed` (plannerd: reason and whether it is slowing), `speed_sign` / `speed_sign_reset` (vision_slc producer:
  limit read, stop sign in view, why the reader restarted), `efficiency_sources` (which fuel inputs the ledger has),
  `edge_map` / `efficiency` recording start and stop summaries, `steer_learner` status/reason. edgemapd and efficiencyd
  now keep recording through a bad frame or failed save. Tests in `starpilot/tests/test_trace_coverage.py`.
- **Self-learning fuel experiments** (2026-11-14): `starpilot/efficiency/experiment.py` (the experimenter: situations
  launch / accelerate / follow / uphill / downhill / ev / pulse_glide / cruise / hold / stop, each with variants; untried
  variants first, then the learned winner 4 in 5 and another 1 in 5; 15 s minimum episodes with entry/exit hysteresis; a
  4 min/engaged-hour time budget (`TIME_BUDGET_S_PER_H` 240 s, 2 min per half hour) for variants that cost time; downhill carry at most `DOWNHILL_CARRY_MPH` 3 mph,
  variant `carry3`; `hold`/`flat` holds the throttle (accel cap 0) in every cruising band, 60+ mph included, until
  `HOLD_BAND_MPH` 2 mph under the set speed; `stop`/`regen` lowers the cruise ceiling to sqrt(2 a d) for one steady
  slowdown at `STOP_REGEN_DECEL` 1 m/s2 (or the gentlest rate that still makes a late stop, up to `STOP_MAX_DECEL`)
  from Force Stop's line or `STOP_GAP_M` behind a stopped car (plannerd passes `stop_distance_m`), so braking stays
  inside regen and the battery counts as fuel, 2026-11-15), `experiment_host.py`
  (plannerd: frame from sm, Pulse and Glide on/off through the coast owner with hand-back and the driver's choice
  respected, status in /dev/shm for the ledger, `efficiency_experiment` decision log; config
  `<efficiency>/experiment.json` mode on/off), `longitudinal_planner.update(efficiency_plan=...)` (lower cruise target,
  the capped downhill carry over an unlimited set speed, a final positive-accel cap; leads/stops still win),
  `ledger` stretches tagged `exp` and split on a variant change, `experiment_learn.py` (battery-adjusted BlueDriver gal
  per mile, or per fixed window for launches; a variant qualifies with enough miles/episodes, 3 % gain and 80 % bootstrap
  confidence over the baseline; only the newest drive's outside-temperature band counts (`SEASON_BANDS` cold/mild/warm:
  summer never weighed against winter); fuel and time are compared per curve class (straight/gentle/curvy, from yaw)
  crossed with the climate band (2026-11-16: off/heat/cool/mild from the car's setpoint, `0x5C4` bytes 0-3 = F - 56,
  0 = off, against the outside temperature; `car_signals.climate_band`; stretches also keep `set_f`, `cabin_f`)
  with equal weights; time-free savings are always taken, then `_spend_budget` spends the 4 min/hr on the steps with the
  most fuel per extra minute across situations, weighted by miles per engaged hour (`situation_miles`), so a dearer step
  that adds little is left out, and no step under `MIN_GAL_PER_MINUTE` (0.02 gal per extra minute, ~$4/hr) is taken;
  road tests count drives from `JUDGE_FROM` 2026-11-15 on (the corrections still fit every drive), and vetoes saved
  before `VETO_EPOCH` 2 are ignored once (the accelerate/launch variants were redefined on 2026-11-14); writes `<efficiency>/experiments.json` (winners, season, budget) at the end of each drive). Tests in
  `starpilot/efficiency/tests/test_experiment*.py`.
- **Turn speed for signalled turns** (2026-11-14): `starpilot/navigation/turn_speed.py`. modeld's blinker-turn logic
  (`intent._timed_turn`) publishes side / turning / distance to the side road / its map angle to
  /dev/shm/starpilot_turn_status.json; plannerd's `TurnSpeedHost` caps the cruise target (`longitudinal_planner.update(
  turn_ceiling_mps=...)`) to ease at 1 m/s2 to the learned turn speed where the turn begins (`blinker_turns.trigger_distance`)
  and hold it through the turn. `TurnLearner` learns the slowest speed through every signalled turn of 60 deg+ (yaw
  rate) per side and sharpness (map angle: shallow < 70, square 70-110, sharp > 110, unknown): the driver's own speed,
  or for comma-driven turns the braked-to / pressed-to speed (no pedal confirms); <storage>/turn_speed.json (seeded
  from 35 recorded turns: right 10.7, left 10.3 mph). Fuel experiments skip while a turn signal is on.
- **Curve memory ignores turns** (2026-11-14): `curve_memory.PassTracker` marks a pass with the blinker on (or on in
  the 3 s before) as verdict `turn`; `curvemapd.Shadow.finish` never learns it (intersection turns had taught tight
  curves at slight mapped bends).
- **Blinker turns trigger earlier** (2026-11-14): `blinker_turns.trigger_distance` = 3.2 s of travel + 7 m (fitted on
  17 recorded map-timed turns; the old 1 s + 3 m started the turn 1.7 s after the driver's steering passed 45 deg).
- **Radar braking shadow trigger** (2026-11-14): `features.LeadAccelShadow` fires below -1.2 m/s2 for 2 radar frames
  (tuned on 392 segments: 90 % real, 8 false/hour, 0.2 s ahead of radard).
- **Pulse and Glide badge** (2026-11-14): `onroad.py` `_pulse_glide_badge` ("P&G: pulse/glide/on/hill pause",
  "Coasting") from `starpilotLongitudinalPlan` (`runtime_snapshot.pulse_glide_badge`; ui_state subscribes it as an
  optional service).
- **Tailscale on boot** (`eb47adb2ca`): `launch_chffrplus.sh` (`tailscale_init`).
- **Partition links for AGNOS 20.5.1**: `launch_chffrplus.sh` (`partition_links`) recreates missing
  `/dev/disk/by-partlabel` links every boot; without `bluetooth_<slot>` the Bluetooth radio never starts (no
  BlueDriver). Drop it once an AGNOS release creates the links again (check `ls /dev/disk/by-partlabel` after boot).
- **NNLC choice repeated every segment**: `selfdrive/controls/controlsd.py` re-logs the drive's `nnlc` choice every
  `NNLC_RELOG_S` (55 s) so each rlog segment carries its steering label. Test: `selfdrive/controls/tests/test_nnlc_relog.py`.
- **Battery reading glitch filter**: `starpilot/efficiency/ledger.py` `_smoothed_soc` holds back a jump of
  `SOC_JUMP` (4) points until `SOC_CONFIRM` (3) readings agree. Tests in `efficiency/tests/test_measured_fuel.py`.
- **Road search off the model process + vectorized** (2026-11-13, after model-lag alerts on the first drive with road
  tiles): `starpilot/navigation/side_road.py` searches are NumPy (`_segments`, `_project_many`, `_endpoints`; identical
  results to the old loops on 300 random positions, ~60x faster). Blinker-turn lookups no longer run inside modeld:
  `MapTurnFinder(remote=True)` (intent.py) writes `/dev/shm/starpilot_side_road_request.json` and the new ancillary
  `sideroadd` answers into `..._result.json`. curvemapd polls modelV2 (20 Hz), looks up every 2 s and only after 20 m.
  Tests: `navigation/tests/test_sideroadd.py`.
- **DR4L upload progress on the home screen**: `starpilot/ui/dr4l_upload.py` reads `<storage>/dr4l_pull.json` (written
  by the DR4L trainer while it pulls) and the compact home screen's third line shows "Uploading to DR4L 42% · 31/74"
  (`home.py` `_compact`, `HomeState.upload`, `runtime_snapshot.py`). Age is measured from the file's last change on the
  comma's monotonic clock; a file present at startup is a baseline, not a live pull. Test: `ui/tests/test_dr4l_upload.py`.
- **Map curve speed (shadow mode)**: `starpilot/curve_speed/map_curves.py` (curves up to 1 km ahead from the stored
  road tiles, highways included, as 10 m chunks via `profile_ahead`), `map_target.py` (chunk-by-chunk plan: each chunk's
  limit is the lowest of learned comfort, 80% of the measured steering torque capability, and a lateral-jerk limit for
  S-bends/quick tightening; a backward pass makes it reachable with a gentle slowdown; curve memory corrects the
  map's tightness per remembered curve; the steering limit uses `steering_capability.py`, learned per speed band
  from live comma steering), a live check in `curvemapd.Shadow._live` (inside a curve, measured tightness/torque vs
  the plan's prediction; logs the mid-curve lowering it would make), `curve_memory.py` (remembers each curve; your own speed sets comfort when the curve set it (no radar lead within
  3 s, you slowed for it); passes behind a lead are lower bounds only and their brake/gas are ignored; each pass is binned into
  11 points through the curve (`pass_profile`) so the comfort curve learns every part at its own tightness and each
  remembered curve keeps its shape and speed profile, which `map_target.adjustments` replays point by point;
  takeovers learn
  the speed actually taken: brake/cancel = too fast, gas =
  too slow, steering into the curve >= 0.5 s or torque saturation = comma struggled; driver torque sign learned from
  hand steering), `curvemapd.py` (ancillary process, logs only, never publishes control; config
  `<storage>/curve_map.json`, logs `<storage>/curve_map/shadow/*.jsonl`). Depends on `side_road._polylines`
  `include_motorway` and `side_road._path`, which follows the straightest branch at a split unless a blinker
  `prefer`s the branch to its side within 600 m (`_take_branch` also catches ramps leaving mid-line); curvemapd passes
  the blinker so exits are planned as soon as you signal. Camera checks (log only, `camera.py` + `curvemapd.CameraCheck`,
  `<storage>/curve_map/camera/*.jsonl`): lane position from road edges/outer lane lines, which branch the model's path
  follows at a split (`map_curves.branches_from_lines`), and map-vs-camera curve agreement; sign conventions are
  learned from driving. Test: `starpilot/curve_speed/tests/test_map_curve_speed.py`.
- **Galaxy remote ready as soon as there is internet**: `starpilot/galaxy/remote_service.py` (`network_route`)
  restarts the frpc tunnel the moment the comma's internet route changes (boot, Wi-Fi to hotspot) instead of
  waiting ~40 s for the dead relay connection's keepalive; `remote.py` `frpc_config` sets
  `tcpMuxKeepaliveInterval = 10` and `dialServerTimeout = 5`. Test: `starpilot/galaxy/tests/test_remote_service.py`.
  If upstream changes the frpc version, re-run `frpc verify` on the generated config (frpc rejects unknown keys).

## Outside the repo (not touched by merges, but must keep working)
- **DR4L trainer** (Unraid, `/mnt/user/appdata/nnlc`, container `nnlc-trainer`): pulls logs over SSH, trains the
  personal NNLC, runs the trial, deploys to `/data/starpilot/nnlc`. Its source is not versioned in this repo yet.
  Extract v4 (2026-11-13): drive mode from the 0x2FC announcement, carried across each drive's segments by
  `extract.resolve_route` (rows store this segment's view, npz `mode_end` the mode at its end); training and scoring
  load rows through `service.route_rows`. Previous code kept in `build/app.v3.bak`.
  Pull progress (2026-11-13): `service.pull` runs rsync in the background and writes `/data/starpilot/dr4l_pull.json`
  on the comma every 5 s (`report_progress`, bytes-weighted percent); v4 code kept in `build/app.v4.bak`.
  It reads rlogs with its own copy of the cereal schema: re-copy `openpilot/cereal/*.capnp` into its
  `build/app/schema` after a merge that changes the schema.
- **Comma files**: `/data/tailscale` (binaries + state), `/data/starpilot/nnlc/` (model, config, user override),
  `/data/starpilot/bluedriver/config.json`, `/data/starpilot/curve_guard.json`, `/data/starpilot/blinker_turns.json`,
  the BlueDriver bond under `/data/bluetooth`, `BluetoothEnabled` param.
- **DR4L SSH key** in the comma's `GithubSshKeys` param: re-fetching keys from GitHub on the comma removes it.
- **Stop-line finder training** (PC, 2026-11-15): drives come from DR4L `/mnt/user/comma` (comma-puller), data in
  `/mnt/Projects/stopline-data`, torch venv in `/mnt/Projects/stopline-train/.venv` (cu128, RTX 5070 Ti). Steps, all
  from the repo root with the repo venv and `PYTHONPATH=.:openpilot:opendbc_repo:msgq_repo` except training:
  1. `python -m openpilot.starpilot.stop_line.extract /mnt/Projects/stopline-data` (rlogs → per-drive tables: full
     stops as first in line, queue stops and parking left out; rolling stops (slowed under 3 m/s without stopping);
     camera calibration per frame; new drives only).
  2. Tag new front stops light / sign / other in `stopline-data/review.json` (by looking at them; lot exits and
     driveways are `other`).
  3. `stopline-train/.venv/bin/python openpilot/starpilot/stop_line/lines.py /mnt/Projects/stopline-data` finds the
     painted line at each full and rolling stop (top-down view from the camera calibration, a band across the lane
     seen from many frames 4-16 m out). Check each in `lines_review.json` (`line_ok`; rolling stops tagged sign / light /
     other; rolling stops at lights are `light` and not used: the light turned green). A checked line places the stop
     3.15 m (camera) before it, the median of where drivers rest; otherwise the rest point is used (worn line).
  4. `python -m openpilot.starpilot.stop_line.frames /mnt/Projects/stopline-data`.
  5. `stopline-train/.venv/bin/python openpilot/starpilot/stop_line/train.py /mnt/Projects/stopline-data <run>`
     (cross-validation by place, ours vs chestnut; `--workers` low while the PC is used for games), then the same
     with `--final` to write `stop_line.onnx`.
  6. Copy it to the comma's `/data/starpilot/stop_line/stop_line.onnx`; `stop_lined` reloads it within 10 s.
  The video-frame index in each segment is `narrowRoadEncodeIdx.segmentId` (this fork leaves `segmentIdEncode` 0).
