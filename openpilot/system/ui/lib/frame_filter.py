"""Animation filters that step by the real time between drawn frames.

FirstOrderFilter takes a fixed dt, and UI code passed 1/60 s. When the UI draws slower than
60 FPS every update still counted as 1/60 s, so fades and smoothing ran slow. These filters
re-read gui_app.frame_dt on each update, keeping their time constants in seconds at any frame rate.
"""
from openpilot.common.filter_simple import BounceFilter, FirstOrderFilter
from openpilot.system.ui.lib.application import gui_app


class _FrameStep:
  dt: float
  rc: float

  def update_alpha(self, rc):
    self.rc = rc
    super().update_alpha(rc)  # type: ignore[misc]

  def _sync_dt(self) -> None:
    dt = gui_app.frame_dt
    if dt != self.dt:
      self.dt = dt
      self.update_alpha(self.rc)


class FrameFilter(_FrameStep, FirstOrderFilter):
  def __init__(self, x0, rc, initialized=True):
    FirstOrderFilter.__init__(self, x0, rc, gui_app.frame_dt, initialized)

  def update(self, x):
    self._sync_dt()
    return FirstOrderFilter.update(self, x)


class FrameBounceFilter(_FrameStep, BounceFilter):
  def __init__(self, x0, rc, initialized=True, bounce=2):
    BounceFilter.__init__(self, x0, rc, gui_app.frame_dt, initialized, bounce)
    self.velocity = FrameFilter(0.0, 0.15)

  def update(self, x):
    self._sync_dt()
    return BounceFilter.update(self, x)
