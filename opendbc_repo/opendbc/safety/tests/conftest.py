"""1337 (2026-11-18): give the release-only denial tests the release safety library.

Two test classes assert what the RELEASE build of the safety code refuses. Upstream runs them only in its release
vehicle plan (tools/ci/run_vehicle_tests.py; tools/ci/safety_pytest_coverage.RELEASE_ONLY keeps them out of the debug
run). In a plain `pytest` run the debug library is loaded and all nine cases fail, which reads like broken safety code.
Here they get a release build for their duration, so they run for real everywhere; under a release library already
loaded nothing is swapped.
"""
import pytest

RELEASE_ONLY = ('test_hyundai_main_rearm.py::TestHkgMainRelease',
                'test_hyundai_classic_cancel_joined.py::TestClassicCancelReleaseDenied')


def _debug_library_loaded() -> bool:
  from opendbc.car.structs import CarParams
  from opendbc.safety.tests.libsafety import libsafety_py
  native = libsafety_py.libsafety
  try:
    return native.set_safety_hooks(CarParams.SafetyModel.allOutput, 0) == 0   # allOutput exists only with ALLOW_DEBUG
  finally:
    native.init_tests()
    native.set_alternative_experience(0)
    native.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)


@pytest.fixture(scope='session')
def _release_library(tmp_path_factory):
  from opendbc.safety.tests.libsafety import libsafety_py
  return libsafety_py._build_libsafety(release=True, output_dir=tmp_path_factory.mktemp('release-safety'))


@pytest.fixture(autouse=True)
def release_library_for_release_only_tests(request):
  if not any(name in request.node.nodeid for name in RELEASE_ONLY) or not _debug_library_loaded():
    yield
    return
  from opendbc.safety.tests.libsafety import libsafety_py
  previous = libsafety_py.libpath
  libsafety_py.load(request.getfixturevalue('_release_library'))
  try:
    yield
  finally:
    libsafety_py.load(previous)
