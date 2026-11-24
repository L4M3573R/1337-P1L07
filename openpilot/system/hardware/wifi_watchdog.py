"""Wi-Fi watchdog: let go of a Wi-Fi network that is gone.

2026-11-24: the comma joined home Wi-Fi at car start, then kept wlan0 as its default route with no signal
(wpa_supplicant SIGNAL_POLL gave no RSSI) for 42 minutes after driving away, so it never tried the phone
hotspot. When Wi-Fi has been the uplink with no signal for STUCK_S, and a slower re-poll agrees, drop
wlan0 and turn its autoconnect back on, so NetworkManager joins whichever saved network is in range.
"""
import subprocess
import threading
from collections.abc import Callable

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength

STUCK_S = 60.0       # Wi-Fi uplink with no signal this long (hardwared samples every 10 s)
COOLDOWN_S = 180.0   # between kicks, so a weak network is not dropped over and over
RECHECKS = 3         # slower SIGNAL_POLLs that must all come back without an RSSI before acting
CMD_TIMEOUT_S = 15.0


def _run(cmd: list[str]) -> bool:
  try:
    return subprocess.run(cmd, capture_output=True, timeout=CMD_TIMEOUT_S, check=False).returncode == 0
  except (OSError, subprocess.SubprocessError):
    return False


def default_kick(run: Callable[[list[str]], bool] = _run) -> bool:
  """Drop wlan0, then re-enable its autoconnect (nmcli's disconnect turns it off) so NM picks a saved network."""
  dropped = run(["nmcli", "--wait", "10", "device", "disconnect", "wlan0"])
  restored = run(["nmcli", "device", "set", "wlan0", "autoconnect", "yes"])
  return dropped and restored


def default_signal_lost() -> bool:
  """Slower re-poll than hardwared's 0.2 s SIGNAL_POLL, so a busy CPU is not read as a lost network."""
  from openpilot.common.hardware.comma.hardware import wpa_supplicant_cmd
  for _ in range(RECHECKS):
    try:
      if wpa_supplicant_cmd("SIGNAL_POLL", timeout=1.0).get("RSSI") is not None:
        return False
    except Exception:
      pass
  return True


class WifiWatchdog:
  def __init__(self, kick: Callable[[], bool] = default_kick, signal_lost: Callable[[], bool] = default_signal_lost,
               threaded: bool = True):
    self.kick, self.signal_lost, self.threaded = kick, signal_lost, threaded
    self.stuck_since: float | None = None
    self.last_kick: float | None = None
    self.kicks = 0
    self._busy = threading.Lock()

  def update(self, network_type, network_strength, now: float) -> bool:
    """Feed every hardware sample; returns True when a kick was started."""
    if network_type != NetworkType.wifi or network_strength != NetworkStrength.unknown:
      self.stuck_since = None
      return False
    if self.stuck_since is None:
      self.stuck_since = now
    if now - self.stuck_since < STUCK_S:
      return False
    if self.last_kick is not None and now - self.last_kick < COOLDOWN_S:
      return False
    if not self._busy.acquire(blocking=False):
      return False
    self.last_kick = now
    self.kicks += 1
    stuck_s = now - self.stuck_since
    if self.threaded:
      threading.Thread(target=self._do, args=(stuck_s,), name="wifi_watchdog", daemon=True).start()
    else:
      self._do(stuck_s)
    return True

  def _do(self, stuck_s: float) -> None:
    try:
      if not self.signal_lost():
        cloudlog.info("wifi_watchdog: signal back on re-poll, not dropping Wi-Fi")
        return
      ok = self.kick()
      msg = f"wifi_watchdog: Wi-Fi uplink had no signal for {stuck_s:.0f} s, dropped wlan0 to rejoin a network in range"
      cloudlog.warning(f"{msg} (ok={ok}, kick {self.kicks})")
    except Exception:
      cloudlog.exception("wifi_watchdog: kick failed")
    finally:
      self._busy.release()
