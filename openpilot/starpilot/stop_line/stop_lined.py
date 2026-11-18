#!/usr/bin/env python3
"""Stop-line finder, shadow mode: where to stop at a light or sign, next to the driving model.

Runs the PC-trained model (<starpilot storage>/stop_line/stop_line.onnx, deployed from the PC like NNLC; no file,
no inference; a corrector model also gets chestnut's planned stop and learns to fix it) on the narrow road camera a few times a second on the little cores, fuses its frames into one stop
point (fusion.py) and picks what a planner would use: ours when sure, else chestnut's planned stop. Nothing reads it
yet; it only logs `starpilot_feature` / `stop_line` events:
  - the choice (source, reason, distances) whenever the source or reason changes, at most once a second;
  - `what=rest` each time the car comes to rest after an approach: where ours and chestnut put the stop at 60, 30 and
    15 m out, against where the car actually stopped (and whether a radar lead was close, i.e. a queue stop),
    so the trace report scores both over real drives before anything is wired to the brakes.
"""
from __future__ import annotations

import math
import os
import time
from collections import deque

import numpy as np

from openpilot.starpilot.diagnostics.features import ChangeLog, log_error, log_event
from openpilot.starpilot.stop_line.fusion import Choice, StopPointFuser, choose, decode
from openpilot.starpilot.stop_line.labels import APPROACH_SPEED_MPS, MIN_STOP_S, QUEUE_LEAD_M, STOP_SPEED_MPS, model_stop_distance

FEATURE = 'stop_line'
INPUT_W, INPUT_H = 448, 256
MIN_INTERVAL_S = 0.25
BUSY_FACTOR = 2.5  # never spend more than 1/BUSY_FACTOR of a little core
MODEL_CHECK_S = 10.0
LABEL_RANGE_M = 120.0
CHECKPOINTS_M = (60.0, 30.0, 15.0)
HISTORY_M = 150.0


def model_path():
  from openpilot.starpilot.storage import starpilot_storage_root
  return starpilot_storage_root() / 'stop_line' / 'stop_line.onnx'


class Shadow:
  """Odometer, fusion, choice and the shadow logs; fed by the daemon, no IPC of its own."""

  def __init__(self, log=None):
    self.log = log
    self.fuser = StopPointFuser()
    self.choices = ChangeLog(FEATURE, log=log, min_gap_s=1.0)
    self.odometer_m = 0.0
    self.last_t: float | None = None
    self.history: deque = deque()  # (odometer, ours left, model left, source)
    self.stopped_since: float | None = None
    self.approached = False
    self.last: Choice | None = None
    self.model_stop_m: float | None = None  # chestnut's latest planned stop, fed to a corrector model

  def inference(self, out, odometer_m: float, now_s: float) -> None:
    self.fuser.update(decode(out), odometer_m, now_s)

  def tick(self, now_s: float, speed_mps: float, model_stop_m: float | None, model_big: bool, model_should_stop: bool,
           ours_available: bool, lead_m: float | None = None) -> Choice:
    if self.last_t is not None:
      self.odometer_m += max(speed_mps, 0.0) * min(max(now_s - self.last_t, 0.0), 0.5)
    self.last_t = now_s
    self.model_stop_m = model_stop_m
    self.fuser.passed(self.odometer_m, speed_mps)
    c = choose(self.fuser, self.odometer_m, now_s, model_stop_m, model_big, model_should_stop, ours_available)
    self.last = c
    self.choices.observe({'source': c.source, 'reason': c.reason},
                         {'distance_m': _r(c.distance_m), 'ours_m': _r(c.ours_m), 'ours_sigma_m': _r(c.ours_sigma_m),
                          'model_m': _r(c.model_m), 'v': round(speed_mps, 2)}, now=now_s)
    self._score(now_s, speed_mps, c, lead_m)
    return c

  def _score(self, now_s: float, speed_mps: float, c: Choice, lead_m: float | None) -> None:
    if speed_mps >= STOP_SPEED_MPS:
      self.stopped_since = None
      if speed_mps > APPROACH_SPEED_MPS:
        self.approached = True
      self.history.append((self.odometer_m, c.ours_m, c.model_m, c.source))
      while self.history and self.odometer_m - self.history[0][0] > HISTORY_M:
        self.history.popleft()
      return
    if self.stopped_since is None:
      self.stopped_since = now_s
    if not self.approached or now_s - self.stopped_since < MIN_STOP_S:
      return
    self.approached = False
    rest = self.odometer_m
    fields = {'what': 'rest', 'queue': lead_m is not None and lead_m < QUEUE_LEAD_M}
    for cp in CHECKPOINTS_M:
      near = min(self.history, key=lambda h: abs(rest - cp - h[0]), default=None)
      if near is None or abs(rest - cp - near[0]) > 5.0:
        continue
      odo, ours, model, source = near
      key = int(cp)
      fields[f'ours_err_{key}'] = _r(odo + ours - rest) if ours is not None else None
      fields[f'chestnut_err_{key}'] = _r(odo + model - rest) if model is not None else None
      fields[f'source_{key}'] = source
    self.history.clear()
    log_event(FEATURE, 'rest', min_gap_s=0.0, log=self.log, now=now_s, **fields)


def chestnut_input(model_stop_m: float | None) -> np.ndarray:
  """The corrector's second input, as in train.chestnut_aux: (has a planned stop in range, distance / range)."""
  has = model_stop_m is not None and math.isfinite(model_stop_m) and 0.0 <= model_stop_m <= LABEL_RANGE_M
  return np.array([[1.0 if has else 0.0, model_stop_m / LABEL_RANGE_M if has else 0.0]], dtype=np.float32)


def _takes_chestnut(net) -> bool:
  """A corrector model has a second input named 'chestnut'; an image-only model fails on it."""
  try:
    net.setInput(np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32), 'image')
    net.setInput(chestnut_input(None), 'chestnut')
    net.forward()
    return True
  except Exception:
    return False


def _r(x: float | None) -> float | None:
  return round(x, 1) if x is not None and math.isfinite(x) else None


def _configure_process() -> None:
  from openpilot.common.hardware import PC
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  from openpilot.starpilot.vision_cpu import LITTLE_CORES
  if PC:
    return
  for configure in (drop_realtime, lambda: set_core_affinity(list(LITTLE_CORES))):
    try:
      configure()
    except OSError:
      pass
  try:
    os.nice(max(0, 15 - os.getpriority(os.PRIO_PROCESS, 0)))
  except (AttributeError, OSError):
    pass


def _frame(client, cv2):
  """Latest narrow road frame as the model's input blob (1x3xHxW RGB 0..1), or None."""
  buf = client.recv(5)
  if buf is None:
    return None
  stride, width, height, uv_offset = int(client.stride), int(client.width), int(client.height), int(client.uv_offset)
  raw = np.frombuffer(buf.data, dtype=np.uint8)
  required = uv_offset + stride * (height // 2)
  if raw.size < required or stride < width:
    return None
  y = raw[:stride * height].reshape((height, stride))[:, :width]
  uv = raw[uv_offset:required].reshape((height // 2, stride))[:, :width]
  rgb = cv2.cvtColor(np.concatenate((y, uv), axis=0), cv2.COLOR_YUV2RGB_NV12)  # copies out of the shared buffer
  rgb = cv2.resize(rgb, (INPUT_W, INPUT_H), interpolation=cv2.INTER_AREA)
  return cv2.dnn.blobFromImage(rgb, scalefactor=1 / 255.0, size=(INPUT_W, INPUT_H), swapRB=False, crop=False)


def main() -> None:
  import cv2
  import openpilot.cereal.messaging as messaging
  from msgq.visionipc import VisionIpcClient
  from openpilot.cereal.visionipc import VisionStreamType

  _configure_process()
  cv2.setNumThreads(1)
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarState'])
  shadow = Shadow()
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
  net, net_mtime, last_check, corrector = None, None, -1e9, False
  next_inference, interval = 0.0, MIN_INTERVAL_S
  while True:
    sm.update(50)
    now = time.monotonic()
    if now - last_check >= MODEL_CHECK_S:
      last_check = now
      try:
        path = model_path()
        mtime = path.stat().st_mtime if path.exists() else None
        if mtime != net_mtime:
          net, net_mtime = (cv2.dnn.readNetFromONNX(str(path)) if mtime is not None else None), mtime
          corrector = net is not None and _takes_chestnut(net)
          shadow.fuser.reset()
          log_event(FEATURE, 'model', min_gap_s=0.0, what='model', loaded=net is not None, corrector=corrector, mtime=mtime)
      except Exception as e:
        net, net_mtime = None, None
        log_error(FEATURE, e, where='load')
    if net is not None and now >= next_inference:
      try:
        if not client.is_connected():
          client.connect(False)
        if client.is_connected():
          odometer = shadow.odometer_m
          blob = _frame(client, cv2)
          if blob is not None:
            t0 = time.monotonic()
            if corrector:
              net.setInput(blob, 'image')
              net.setInput(chestnut_input(shadow.model_stop_m), 'chestnut')
            else:
              net.setInput(blob)
            out = net.forward().reshape(-1)[:3]
            took = time.monotonic() - t0
            shadow.inference(out, odometer, time.monotonic())
            interval = max(MIN_INTERVAL_S, BUSY_FACTOR * took)
      except Exception as e:
        log_error(FEATURE, e, where='inference')
      next_inference = now + interval
    if sm.updated['modelV2']:
      md, cs, lead = sm['modelV2'], sm['carState'], sm['radarState'].leadOne
      stop = model_stop_distance(md.position.x, md.velocity.x)
      shadow.tick(now, cs.vEgo, stop, md.big, md.action.shouldStop, net is not None,
                  lead.dRel if lead.present else None)


if __name__ == '__main__':
  main()
