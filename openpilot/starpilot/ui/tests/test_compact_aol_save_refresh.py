"""Compact driving controls redraw after a worker-thread AOL save settles."""
import unittest
from unittest.mock import Mock, patch

from openpilot.starpilot.ui.feature_settings_state import FeatureRow


class Button:
  def __init__(self, text, detail):
    self.text, self.detail, self.click = text, detail, None

  def set_click_callback(self, click):
    self.click = click


class FakeApp:
  def __init__(self, page):
    self.stack, self.ticks = [page], []

  def widget_in_stack(self, widget):
    return widget in self.stack

  def add_nav_stack_tick(self, tick):
    if tick not in self.ticks:
      self.ticks.append(tick)

  def remove_nav_stack_tick(self, tick):
    if tick in self.ticks:
      self.ticks.remove(tick)

  def frame(self):
    for tick in list(self.ticks):
      tick()


def aol_row():
  return FeatureRow("AlwaysOnLateral", "Enable Always On Lateral", "Off", b"0", ("Off", "On"), available=True,
                    vehicle_fingerprint="HYUNDAI_ELANTRA_HEV_2024", capability=("capability",))


class Session:
  """Mirrors StarShellSession: AOL saves report False and finish on a worker."""

  def __init__(self):
    self.requests, self.pending = [], False

  def feature_snapshot(self, page):
    raise AssertionError("not used")

  def feature_request(self, request):
    self.requests.append(request)
    self.pending = True
    return False

  def feature_save_pending(self):
    return self.pending


class TestCompactAolSaveRefresh(unittest.TestCase):
  def setUp(self):
    from openpilot.starpilot.ui import feature_settings_compact as compact
    self.compact = compact
    self.page = Mock(spec=compact.NavScroller)
    self.app = FakeApp(self.page)

  def tap(self, session):
    refreshed = []
    adapter = self.compact.FeatureSettingsCompact(session)
    with patch.object(self.compact, "BigButton", Button), patch.object(self.compact, "gui_app", self.app):
      (card,) = adapter._editable(aol_row(), lambda: refreshed.append(True), self.page)
      card.click()
    return refreshed

  def test_pending_save_redraws_now_and_after_it_settles(self):
    session = Session()
    refreshed = self.tap(session)
    self.assertEqual(session.requests[0].value, "On")
    self.assertEqual(session.requests[0].expected, b"0")
    self.assertEqual(refreshed, [True])  # shows "saving" immediately
    with patch.object(self.compact, "gui_app", self.app):
      self.app.frame()
      self.assertEqual(refreshed, [True])  # still saving
      session.pending = False
      self.app.frame()
      self.assertEqual(refreshed, [True, True])  # shows the saved value
      self.assertEqual(self.app.ticks, [])
      self.app.frame()
    self.assertEqual(refreshed, [True, True])

  def test_leaving_the_page_stops_watching(self):
    session = Session()
    refreshed = self.tap(session)
    self.app.stack.clear()
    with patch.object(self.compact, "gui_app", self.app):
      self.app.frame()
    self.assertEqual(self.app.ticks, [])
    self.assertEqual(refreshed, [True])

  def test_rejected_synchronous_save_does_not_redraw(self):
    session = Mock(spec=self.compact.FeatureSession, feature_request=lambda request: False)
    self.assertEqual(self.tap(session), [])
    self.assertEqual(self.app.ticks, [])


if __name__ == "__main__":
  unittest.main()
