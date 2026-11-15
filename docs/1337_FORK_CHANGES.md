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
- **CEM stops** (`dac4e3ce32`, `1102863c55`): `starpilot/conditional_mode/stop_release.py`, `stop.py`.

### Lateral
- **Personal NNLC with automatic trial** (`6c339a40be`, `c5e36c122d`): `starpilot/lateral/nnlc/`,
  `latcontrol_torque.py` (NNLC hook after friction), `controlsd.py` (`_select_nnlc`; FLM off on NNLC drives).
  Trained off-car by the DR4L container (see "Outside the repo").
- **Curve guard (opt-in)** (`9110e41e79`): `starpilot/lateral/curve_guard.py`, `controlsd.py` (`_apply_curve_guard`).

### Turns
- **Blinker turn guidance** (`36a3dc1aa9`, `d2ecbfb5c2`, `42de700f0b`, `4f1ebefe17`): `starpilot/navigation/intent.py`,
  `starpilot/ui/onroad_lane_alerts.py` (Turning Left/Right).
- **Turns timed to the road** (`2792a5f492`): `navigation/side_road.py`, `navigation/blinker_turns.py`
  (shadow/map/speed; default shadow), `intent.py` (turn beats a queued lane change below 20 mph), `navtilesd.py`
  (nearby tiles), `modeld.py` (GPS subscriptions), green blinker on Turning in `alert_renderer.py` and
  `starpilot/ui/onroad_alerts.py`.
- **No-blinker straight guard, shadow only** (2026-11-15): `navigation/straight_guard.py`, run in `sideroadd.py`
  (`guard_step`), map side `side_road.crossing` (nearest junction, its branches, whether the car's road goes on straight)
  through `MapTurnFinder.crossing_now`. Logs `starpilot_feature` / `straight_guard` events (`would_hold`, `outcome`,
  `no_map`) when the model starts turning onto a mapped side road with no blinker, the wheel still near centre and the
  driver not steering, while the road continues straight. Publishes no control; a later active mode would act on the
  steering output, because the blinker-turn lever (desire) is already none without a blinker.

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
  allows (`TIME_BUDGET_MIN_PER_HR` 2.5, at most 5 mph under), and what running up to 2 mph over the set speed on open
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
  launch / accelerate / follow / uphill / downhill / ev / pulse_glide / cruise / hold, each with variants; untried
  variants first, then the learned winner 4 in 5 and another 1 in 5; 15 s minimum episodes with entry/exit hysteresis; a
  2.5 min/engaged-hour time budget for variants that cost time; downhill carry at most `DOWNHILL_CARRY_MPH` 3 mph,
  variant `carry3`; `hold`/`flat` holds the throttle (accel cap 0) in every cruising band, 60+ mph included, until
  `HOLD_BAND_MPH` 2 mph under the set speed, 2026-11-15), `experiment_host.py`
  (plannerd: frame from sm, Pulse and Glide on/off through the coast owner with hand-back and the driver's choice
  respected, status in /dev/shm for the ledger, `efficiency_experiment` decision log; config
  `<efficiency>/experiment.json` mode on/off), `longitudinal_planner.update(efficiency_plan=...)` (lower cruise target,
  the capped downhill carry over an unlimited set speed, a final positive-accel cap; leads/stops still win),
  `ledger` stretches tagged `exp` and split on a variant change, `experiment_learn.py` (battery-adjusted BlueDriver gal
  per mile, or per fixed window for launches; winner needs enough miles/episodes, 3 % gain and 80 % bootstrap
  confidence; writes `<efficiency>/experiments.json` at the end of each drive). Tests in
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
