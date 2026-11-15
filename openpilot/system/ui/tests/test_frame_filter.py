import pytest

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.system.ui.lib import frame_filter
from openpilot.system.ui.lib.application import gui_app


def run(filter_cls, fps, seconds, target=1.0, monkeypatch=None, **kwargs):
  monkeypatch.setattr(gui_app, "_frame_dt", 1 / fps)
  f = filter_cls(0.0, 0.1, **kwargs)
  for _ in range(round(seconds * fps)):
    f.update(target)
  return f


@pytest.mark.parametrize("fps", [20, 30])
def test_frame_filter_keeps_its_time_constant_in_seconds(fps, monkeypatch):
  reference = run(frame_filter.FrameFilter, 60, 0.3, monkeypatch=monkeypatch).x
  slower = run(frame_filter.FrameFilter, fps, 0.3, monkeypatch=monkeypatch).x
  assert slower == pytest.approx(reference, abs=0.05)
  # The fixed 1/60 s step this replaces falls far behind at the same frame rate.
  fixed = FirstOrderFilter(0.0, 0.1, 1 / 60)
  for _ in range(round(0.3 * fps)):
    fixed.update(1.0)
  assert reference - fixed.x > 0.15


def test_frame_filter_follows_frame_rate_changes(monkeypatch):
  monkeypatch.setattr(gui_app, "_frame_dt", 1 / 60)
  f = frame_filter.FrameFilter(0.0, 0.1)
  f.update(1.0)
  assert f.alpha == pytest.approx((1 / 60) / (0.1 + 1 / 60))
  monkeypatch.setattr(gui_app, "_frame_dt", 1 / 20)
  f.update(1.0)
  assert (f.dt, f.rc) == (1 / 20, 0.1)
  assert f.alpha == pytest.approx(0.05 / 0.15)
  f.update_alpha(0.2)  # a caller retuning rc keeps it across later frame-rate changes
  monkeypatch.setattr(gui_app, "_frame_dt", 1 / 30)
  f.update(1.0)
  assert f.alpha == pytest.approx((1 / 30) / (0.2 + 1 / 30))


def test_uninitialized_frame_filter_takes_the_first_value(monkeypatch):
  monkeypatch.setattr(gui_app, "_frame_dt", 1 / 20)
  f = frame_filter.FrameFilter(0.0, 0.1, initialized=False)
  assert f.update(5.0) == 5.0


@pytest.mark.parametrize("fps", [20, 30])
def test_frame_bounce_filter_settles_like_it_does_at_60_fps(fps, monkeypatch):
  reference = run(frame_filter.FrameBounceFilter, 60, 1.0, monkeypatch=monkeypatch)
  slower = run(frame_filter.FrameBounceFilter, fps, 1.0, monkeypatch=monkeypatch)
  assert isinstance(slower.velocity, frame_filter.FrameFilter)
  assert slower.x == pytest.approx(reference.x, abs=0.05)
  assert slower.x == pytest.approx(1.0, abs=0.05)
