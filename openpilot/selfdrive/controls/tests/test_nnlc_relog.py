from types import SimpleNamespace

from openpilot.selfdrive.controls import controlsd


class _SM:
  updated = {'extrinsicsCalibration': False, 'deviceMotion': False}

  def update(self, timeout):
    pass


def test_drive_choice_is_repeated_about_once_a_segment(monkeypatch):
  logged = []
  monkeypatch.setattr(controlsd.cloudlog, 'event', lambda *a, **k: logged.append(k))
  now = [1000.0]
  monkeypatch.setattr(controlsd.time, 'monotonic', lambda: now[0])
  stub = SimpleNamespace(sm=_SM(), _nnlc_event={'mode': 'nnlc', 'reason': 'trial', 'model_sha256': 'abc', 'drive': 3},
                         _nnlc_logged=1000.0)
  controlsd.Controls.update(stub)
  assert logged == []
  now[0] += controlsd.NNLC_RELOG_S
  controlsd.Controls.update(stub)
  assert logged == [{'feature': 'nnlc', 'mode': 'nnlc', 'reason': 'trial', 'model_sha256': 'abc', 'drive': 3}]
  controlsd.Controls.update(stub)
  assert len(logged) == 1


def test_no_choice_no_log(monkeypatch):
  logged = []
  monkeypatch.setattr(controlsd.cloudlog, 'event', lambda *a, **k: logged.append(k))
  controlsd.Controls.update(SimpleNamespace(sm=_SM()))
  assert logged == []
