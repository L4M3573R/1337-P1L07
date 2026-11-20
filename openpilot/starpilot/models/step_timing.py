"""Where a model frame's time goes, without logging every frame.

The runner times five steps of each frame with perf_counter_ns (no device syncs, nothing waits because of it):
  prepare  host inputs written (desire, numpy inputs)
  warp     the camera warp enqueued
  policy   the model enqueued (on the Chestnut: commands and inputs handed to the USB link)
  wait     outputs read back (the device finishes its queue here: transfer, GPU work, download)
  parse    outputs checked and parsed
modeld adds each frame's step times here and one `modeld.steps` event goes out every REPORT_S with p50 / p90 / max per
step in milliseconds, next to modelExecutionTime over the same frames. Read it from the rlog's logMessage events.
"""
from __future__ import annotations

import time
from collections.abc import Callable

STEPS = ("prepare", "warp", "policy", "wait", "parse")
REPORT_S = 30.0


def _stats(values: list[float]) -> dict[str, float]:
  ordered = sorted(values)
  pick = lambda q: ordered[min(len(ordered) - 1, int(q * len(ordered)))]
  return {"p50": round(pick(0.50), 2), "p90": round(pick(0.90), 2), "max": round(ordered[-1], 2)}


class StepTiming:
  def __init__(self, report: Callable[..., None], period_s: float = REPORT_S,
               clock: Callable[[], float] = time.monotonic):
    self.report, self.period_s, self.clock = report, period_s, clock
    self.rows: list[tuple[int, ...]] = []
    self.totals: list[float] = []
    self.key: tuple | None = None
    self.started = clock()

  def add(self, step_ns: tuple[int, ...] | None, total_s: float, model_id: str, chestnut: bool) -> None:
    """One frame: the runner's five step times (ns) and modelExecutionTime (s). A frame without them only counts in total."""
    key = (model_id, chestnut)
    if key != self.key:   # a fallback or hand-over mid-window: never mix two models in one report
      self.flush()
      self.key = key
    if step_ns is not None and len(step_ns) == len(STEPS):
      self.rows.append(step_ns)
    self.totals.append(total_s)
    if self.clock() - self.started >= self.period_s:
      self.flush()

  def flush(self) -> None:
    rows, totals, key = self.rows, self.totals, self.key
    self.rows, self.totals, self.started = [], [], self.clock()
    if not totals or key is None:
      return
    packet = {"modelId": key[0], "chestnut": key[1], "frames": len(totals), "timedFrames": len(rows),
              "total": _stats([t * 1e3 for t in totals])}
    for k, step in enumerate(STEPS):
      if rows:
        packet[step] = _stats([r[k] / 1e6 for r in rows])
    self.report(**packet)
