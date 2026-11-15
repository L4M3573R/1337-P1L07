"""Camera ownership regression; native imports, synthetic resource handles.

No graphics context is opened. Release functions record ownership operations,
so this checks close/destructor behavior rather than native GPU rendering.
"""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from openpilot.common.basedir import BASEDIR


class TestCameraResources(unittest.TestCase):
  def test_close_then_destructor_releases_each_resource_once(self):
    script = '''
from importlib import import_module
from types import SimpleNamespace
from unittest.mock import patch

for path in ("openpilot.selfdrive.ui.onroad.cameraview", "openpilot.selfdrive.ui.mici.onroad.cameraview"):
  module = import_module(path)
  owner = import_module(module.CameraView._clear_textures.__module__)
  for hardware in (False, True):
    view = object.__new__(module.CameraView)
    view.texture_y, view.texture_uv = SimpleNamespace(id=11), SimpleNamespace(id=12)
    view.shader = SimpleNamespace(id=19)
    view.egl_texture = SimpleNamespace(id=13) if hardware else None
    view.egl_images = {1: "owned-image"} if hardware else {}
    view.frame, view.client, view.available_streams = None, None, set()
    view._clients = {}
    textures, shaders, images = [], [], []
    with patch.object(module, "COMMA_HARDWARE", hardware), \\
         patch.object(owner, "COMMA_HARDWARE", hardware), \\
         patch.object(module.rl, "unload_texture", side_effect=lambda texture: textures.append(texture.id)), \\
         patch.object(module.rl, "unload_shader", side_effect=lambda shader: shaders.append(shader.id)), \\
         patch.object(owner, "destroy_egl_image", side_effect=images.append):
      try:
        view.close()
        view.close()
        view.__del__()
        assert textures == ([11, 12, 13] if hardware else [11, 12]), (path, hardware, textures)
        assert shaders == [19], (path, hardware, shaders)
        assert images == (["owned-image"] if hardware else []), (path, hardware, images)
      finally:
        # Keep the synthetic handle out of real native cleanup even on failure.
        view.shader.id = 0
        view.texture_y = view.texture_uv = view.egl_texture = None
        view.egl_images.clear()
    del view
'''
    self._run_isolated(script, "close")

  def test_egl_image_is_reattached_only_when_it_changes(self):
    script = '''
from importlib import import_module
from types import SimpleNamespace
from unittest.mock import patch

module = import_module("openpilot.selfdrive.ui.onroad.cameraview")
view = object.__new__(module.CameraView)
view.texture_y = view.texture_uv = None
view.shader = SimpleNamespace(id=0)
view.egl_texture = SimpleNamespace(id=13, width=0, height=0)
view.egl_images = {1: "image-1", 2: "image-2"}
view.frame = SimpleNamespace(idx=1, width=4, height=2)
view.client, view.available_streams, view._clients = None, set(), {}
attached = []
rect = module.rl.Rectangle(0, 0, 1, 1)
try:
  with patch.object(module, "bind_egl_image_to_texture", side_effect=lambda _id, image, *, attach: attached.append((image, attach))), \\
       patch.object(module, "COMMA_HARDWARE", True), patch.object(module, "destroy_egl_image"), \\
       patch.object(module.rl, "begin_shader_mode"), patch.object(module.rl, "end_shader_mode"), \\
       patch.object(module.rl, "draw_texture_pro"), patch.object(view, "_update_texture_color_filtering"):
    for idx in (1, 1, 2, 2, 1):
      view.frame.idx = idx
      view._render_egl(rect, rect)
    assert attached == [("image-1", True), ("image-1", False), ("image-2", True), ("image-2", False), ("image-1", True)], attached
    # Destroyed images must be attached again even when the replacement reuses the buffer index.
    view._clear_textures()
    view.egl_images = {1: "image-1b"}
    view._render_egl(rect, rect)
    assert attached[-1] == ("image-1b", True), attached
finally:
  view.egl_texture, view.egl_images, view.frame = None, {}, None
del view
'''
    self._run_isolated(script, "egl")

  def test_camera_pacer_waits_only_while_the_view_is_drawn(self):
    script = '''
from importlib import import_module
from types import SimpleNamespace
from unittest.mock import Mock, patch

for path in ("openpilot.selfdrive.ui.onroad.cameraview", "openpilot.selfdrive.ui.mici.onroad.cameraview"):
  module = import_module(path)
  base = import_module("openpilot.selfdrive.ui.onroad.cameraview")
  view = object.__new__(module.CameraView)
  view._switching, view.frame, view._texture_needs_update = False, None, False
  view.client = Mock()
  view.client.is_connected.return_value = True
  view.client.recv.return_value = SimpleNamespace(idx=3)
  with patch.object(base.gui_app, "_frame", 10):
    view._polled_frame = 8  # not drawn last frame: menus and other pages keep full speed
    assert view._pace_to_camera() is False and not view.client.recv.called
    view._polled_frame = 9
    assert view._pace_to_camera() is True
    view.client.recv.assert_called_once_with(timeout_ms=base.PACE_TIMEOUT_MS)
    assert view.frame.idx == 3 and view._texture_needs_update
    view.client.recv.return_value = None  # timed out: draw anyway, keep the last frame
    assert view._pace_to_camera() is True and view.frame.idx == 3
    view._switching = True
    assert view._pace_to_camera() is False
    view._switching = False
    view.client.is_connected.return_value = False
    assert view._pace_to_camera() is False
  view.client = view.frame = None
  del view
'''
    self._run_isolated(script, "pace")

  def _run_isolated(self, script, name):
    ipc_root = "/tmp" if sys.platform == "darwin" else "/dev/shm"
    with tempfile.TemporaryDirectory() as temporary, tempfile.TemporaryDirectory(prefix=f"msgq_camera-{name}-", dir=ipc_root) as ipc:
      environment = dict(os.environ, PARAMS_ROOT=str(Path(temporary) / "params"),
                         OPENPILOT_PREFIX=Path(ipc).name.removeprefix("msgq_"))
      for key in ("CEREAL_FAKE", "CEREAL_FAKE_PREFIX", "SIMULATION"):
        environment.pop(key, None)
      if sys.platform == "darwin":
        environment.pop("RAYLIB_BACKEND", None)
      result = subprocess.run([sys.executable, "-c", script], cwd=BASEDIR, env=environment, capture_output=True, text=True, timeout=30)
      self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
  unittest.main()
