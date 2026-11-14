"""plannerd's side of the fuel experiments (experiment.py): builds the experimenter's frame from the planner's inputs,
applies Pulse and Glide on/off through the wheel-coast owner, publishes the episode for the fuel ledger and logs
every change to the decision log. Everything else the plan carries (a lower cruise target, the downhill carry, an
acceleration cap) is applied by the longitudinal planner, after its own limits.

Config <efficiency>/experiment.json: {"mode": "on"} (default) or {"mode": "off"}; read once per drive.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from openpilot.starpilot.efficiency.experiment import STATUS_PATH, Experimenter, Frame, Plan, load_winners, write_status

KPH_TO_MS = 1 / 3.6
SOC_READ_S = 1.0
STATUS_WRITE_S = 0.5
DRIVER_PULSE_GLIDE = 'driver toggled pulse and glide'


def efficiency_root() -> Path:
  from openpilot.starpilot.efficiency.store import default_root
  return default_root()


def mode(root: Path) -> str:
  try:
    value = json.loads((root / 'experiment.json').read_text()).get('mode')
  except (OSError, ValueError, TypeError, AttributeError):
    return 'on'
  return value if value in ('on', 'off') else 'on'


class ExperimentHost:
  def __init__(self, root: Path | None = None, status_path: Path = STATUS_PATH, clock=time.monotonic):
    self.root = root
    self.status_path = status_path
    self.clock = clock
    self.drive = None
    self.enabled = False
    self.exp = Experimenter()
    self.driver_pulse_glide = False
    self.we_turned_pulse_glide_on = False
    self.soc: float | None = None
    self.next_soc = 0.0
    self.next_status = 0.0
    self.log = None
    self.last_tag: str | None = None

  def _new_drive(self, drive) -> None:
    root = self.root if self.root is not None else efficiency_root()
    self.drive = drive
    self.enabled = mode(root) == 'on'
    self.exp = Experimenter(winners=load_winners(root / 'experiments.json'))
    self.driver_pulse_glide = False
    self.we_turned_pulse_glide_on = False
    from openpilot.starpilot.diagnostics.features import ChangeLog
    self.log = ChangeLog('efficiency_experiment', min_gap_s=1.0)

  def _read_soc(self, now: float) -> None:
    if now < self.next_soc:
      return
    self.next_soc = now + SOC_READ_S
    try:
      from openpilot.starpilot.obd.bluedriverd import read_live
      doc = read_live(now=now)
      soc = doc.get('soc') if isinstance(doc, dict) and doc.get('connected') else None
      self.soc = float(soc) if isinstance(soc, (int, float)) and 0 <= soc <= 100 else None
    except Exception:
      self.soc = None

  def sample(self, sm, wheel_coast, *, drive, force_stop_forcing: bool) -> Plan | None:
    """The plan for this cycle, or None when experiments are off or cannot run."""
    try:
      if drive != self.drive:
        self._new_drive(drive)
      if not self.enabled:
        return None
      now = self.clock()
      self._read_soc(now)
      owner = getattr(wheel_coast, 'owner', None)
      if owner is not None and getattr(owner, 'change_reason', '') == DRIVER_PULSE_GLIDE:
        self.driver_pulse_glide = True   # the driver chose: Pulse and Glide stays theirs for the rest of the drive
      cs, cc = sm['carState'], sm['carControl']
      lead = sm['radarState'].leadOne
      v_ego = float(cs.vEgo)
      gap = float(lead.dRel) / max(v_ego, 1.0) if lead.present else math.inf
      pitch = cc.orientationNED[1] if len(cc.orientationNED) == 3 else None
      grade = math.tan(pitch) if pitch is not None and math.isfinite(pitch) else None
      stop_ahead = bool(force_stop_forcing or sm['modelV2'].action.shouldStop or sm['controlsState'].forceDecel or
                        (lead.present and lead.vLead < 1.0 and lead.dRel < 60.0))
      frame = Frame(now, v_ego, float(cs.vCruise) * KPH_TO_MS, bool(cc.longActive and cc.enabled), bool(cs.gasPressed),
                    bool(cs.brakePressed), bool(cs.standstill), grade, gap, stop_ahead, self.soc, self.driver_pulse_glide)
      plan = self.exp.sample(frame)
      self._apply_pulse_glide(plan, owner)
      if now >= self.next_status or plan.tag != self.last_tag:
        self.next_status = now + STATUS_WRITE_S
        self.last_tag = plan.tag
        write_status({'t': now, **self.exp.status()}, self.status_path)
      if self.log is not None:
        status = self.exp.status()
        self.log.observe({'tag': plan.tag}, {'spent_s': status['spent_s'], 'budget_left_s': status['budget_left_s'],
                                             'v_mph': round(v_ego * 2.237, 1)}, now=now)
      return plan
    except Exception as e:
      from openpilot.starpilot.diagnostics.features import log_error
      log_error('efficiency_experiment', e, where='host')
      return None

  def _apply_pulse_glide(self, plan: Plan, owner) -> None:
    if owner is None or self.driver_pulse_glide:
      return
    if plan.pulse_glide is True and not owner.pulse_glide:
      owner.pulse_glide = True
      owner.change_reason = 'efficiency experiment'
      self.we_turned_pulse_glide_on = True
    elif plan.pulse_glide is not True and self.we_turned_pulse_glide_on:
      # the experiment ended: hand back what the driver had (off) unless they took it over meanwhile
      if owner.pulse_glide:
        owner.pulse_glide = False
        owner.change_reason = 'efficiency experiment ended'
      self.we_turned_pulse_glide_on = False
