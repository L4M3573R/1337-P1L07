import json

from openpilot.starpilot.ui.dr4l_upload import DONE_SHOW_S, STALE_S, UploadStatus, upload_text


def test_progress_line_while_pulling():
  doc = {'done': 31, 'total': 74, 'percent': 41.6, 'state': 'pulling'}
  assert upload_text(doc, 3.0) == 'DR4L Uploading: 42%'


def test_stalled_pull_disappears_and_done_shows_briefly():
  pulling = {'done': 5, 'total': 10, 'percent': 50.0, 'state': 'pulling'}
  assert upload_text(pulling, STALE_S + 1) == ''
  done = {'done': 10, 'total': 10, 'percent': 100.0, 'state': 'done'}
  assert upload_text(done, 30.0) == 'DR4L Uploading: 100%'
  assert upload_text(done, DONE_SHOW_S + 1) == ''
  # a pull is never shown as 100% until it is done
  assert upload_text({**pulling, 'percent': 100.0, 'done': 10}, 1.0) == 'DR4L Uploading: 99%'


def test_bad_documents_show_nothing():
  assert upload_text(None, 0.0) == ''
  assert upload_text({'done': 'x', 'total': 2}, 0.0) == ''
  assert upload_text({'done': 3, 'total': 2}, 0.0) == ''
  assert upload_text({'done': 0, 'total': 0}, 0.0) == ''


def test_reader_ages_from_the_last_change_on_its_own_clock(tmp_path):
  path = tmp_path / 'dr4l_pull.json'
  now = [100.0]
  status = UploadStatus(path, clock=lambda: now[0])
  assert status.text() == ''
  path.write_text(json.dumps({'updated': 1.0, 'done': 2, 'total': 4, 'percent': 50.0, 'state': 'pulling'}))
  assert status.text() == ''          # still the cached (missing) read
  now[0] += 2.5
  assert status.text() == 'DR4L Uploading: 50%'
  now[0] += STALE_S + 3               # the file has not changed since: the pull stalled
  assert status.text() == ''
  path.write_text(json.dumps({'updated': 2.0, 'done': 3, 'total': 4, 'percent': 75.0, 'state': 'pulling'}))
  now[0] += 2.5
  assert status.text() == 'DR4L Uploading: 75%'
  path.write_text('{broken')
  now[0] += 2.5
  assert status.text() == ''


def test_an_old_file_at_startup_is_not_shown_until_it_changes(tmp_path):
  path = tmp_path / 'dr4l_pull.json'
  path.write_text(json.dumps({'done': 3, 'total': 10, 'percent': 30.0, 'state': 'pulling', 'seq': 7}))
  now = [100.0]
  status = UploadStatus(path, clock=lambda: now[0])
  assert status.text() == ''          # left over from an earlier pull
  path.write_text(json.dumps({'done': 4, 'total': 10, 'percent': 40.0, 'state': 'pulling', 'seq': 8}))
  now[0] += 2.5
  assert status.text() == 'DR4L Uploading: 40%'
