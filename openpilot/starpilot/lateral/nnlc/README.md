# Personal NNLC feedforward

A small network trained off the car on this car's own drives predicts the steering torque a lateral
acceleration path takes. On drives the trial selects it replaces only the torque controller's
feedforward (friction included). The PID correction, `steer_max`, controlsd and Panda safety are unchanged.

## Guards

- The network may move the feedforward at most 0.3 (normalized torque) from StarPilot's own, and never
  past ±1.0.
- Between 1.5 and 2.2 m/s² it is blended back toward StarPilot's feedforward whenever it would steer
  less (training has few moments that tight, and the network flattens there).
- Non-finite inputs or output, a missing plan or a bad plan length use StarPilot's feedforward for that tick.
- A missing, malformed, oversized or checksum-mismatched model or config means a standard drive.
- Steering learning (FLM) is not started on NNLC drives, so it never learns from the network.

## Files (`<starpilot storage>/nnlc/`, written by the off-car trainer)

- `config.json`: `{"version": 1, "mode": "off" | "trial" | "on", "model": "models/<file>.json", "model_sha256": "<hex>"}`
- `models/<file>.json`: input_size 18, output_size 1, input_mean/std, dense layers (σ hidden, identity output)
- `state.json`: written on the car; in `trial` drives alternate NNLC / standard, a new model starts on NNLC

Each drive logs `starpilot_feature` `feature=nnlc` with the mode, reason, model checksum and drive number.

## Inputs (model order)

speed; lateral accel at `lat_delay`; 0.4 × lateral jerk; roll; that lateral accel 0.3/0.2/0.1 s ago;
planned lateral accel at `lat_delay` + 0.3/0.6/1.0/1.5 s; roll 0.3/0.2/0.1 s ago; predicted roll at those
future times. Training uses the car's actual response at the same offsets after `lat_delay`, so the
controller's own request never grades itself.
