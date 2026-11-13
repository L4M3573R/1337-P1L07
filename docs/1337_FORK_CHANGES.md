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
  `ClusterRatioFilter`), `selfdrive/controls/lib/longitudinal_planner.py`.
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

### Efficiency and fuel
- **Energy ledger** (`246ace1369`, `9eafb2765a`, `eb7c0ce812`, `9355d3146c`, `575dc06820`, `835eb17fa5`):
  `starpilot/efficiency/` (efficiencyd ancillary in `process_config.py` and `process_health.py`), dash fuel,
  hybrid power flow (0x595), drive mode / tire pressure / map limit tags (`car_signals.py`), per-stretch records.
- **BlueDriver Pro logger** (`eb47adb2ca`): `starpilot/obd/` (bluedriverd, opt-in, ancillary), measured fuel and
  battery in the ledger, tank level, read-only health check.
- **Comma vs driver and PC reports** (`575dc06820`, `835eb17fa5`): `efficiency/compare.py`, `trend.py`, `trips.py`,
  `pc_summary.py`.

### Settings and UI
- **L4 custom tile** (`c5e36c122d`, `e825ae80af`, `b91e0d2fa9`, `eb47adb2ca`, `575dc06820`):
  `selfdrive/ui/mici/layouts/settings/settings.py`, `.../l4_custom.py`, `starpilot/ui/l4_custom.py`.
- **Compact driving controls redraw after AOL saves** (`92fdf51393`).

### Other
- **Edge (curb) map recorder** (`fb1026580a`): `starpilot/edgemap/`, edgemapd ancillary.
- **Chestnut/model loading** (`07cbcf0f22`, `dab4cd3c35`, `abc4eddd03`): `starpilot/models/runner.py`,
  `runtime_bridge/runtime.py`, `modeld.py`.
- **Decision logs** (`4f1ebefe17`, `2792a5f492`): `starpilot/diagnostics/features.py`, reserved
  `starpilotLongitudinalPlan` in `cereal/custom.capnp`.
- **Tailscale on boot** (`eb47adb2ca`): `launch_chffrplus.sh` (`tailscale_init`).

## Outside the repo (not touched by merges, but must keep working)
- **DR4L trainer** (Unraid, `/mnt/user/appdata/nnlc`, container `nnlc-trainer`): pulls logs over SSH, trains the
  personal NNLC, runs the trial, deploys to `/data/starpilot/nnlc`. Its source is not versioned in this repo yet.
  It reads rlogs with its own copy of the cereal schema: re-copy `openpilot/cereal/*.capnp` into its
  `build/app/schema` after a merge that changes the schema.
- **Comma files**: `/data/tailscale` (binaries + state), `/data/starpilot/nnlc/` (model, config, user override),
  `/data/starpilot/bluedriver/config.json`, `/data/starpilot/curve_guard.json`, `/data/starpilot/blinker_turns.json`,
  the BlueDriver bond under `/data/bluetooth`, `BluetoothEnabled` param.
- **DR4L SSH key** in the comma's `GithubSshKeys` param: re-fetching keys from GitHub on the comma removes it.
