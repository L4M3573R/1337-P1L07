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

from openpilot.starpilot.efficiency.experiment import (BUDGET_FILE, STATUS_PATH, STOP_GAP_M, DialExperimenter, Experimenter,
                                                       Frame, Plan, load_budget, load_vetoes, load_winners, save_vetoes,
                                                       write_status)

KPH_TO_MS = 1 / 3.6
SOC_READ_S = 1.0
STATUS_WRITE_S = 0.5
RELOAD_CHECK_S = 5.0   # efficiencyd judges the stored drives as the drive starts: pick its result up when it lands
DRIVER_PULSE_GLIDE = 'driver toggled pulse and glide'


def efficiency_root() -> Path:
  from openpilot.starpilot.efficiency.store import default_root
  return default_root()


def mode(root: Path) -> str:
  try:
    value = json.loads((root / 'experiment.json').read_text()).get('mode')
  except (OSError, ValueError, TypeError, AttributeError):
    return 'off'   # no (readable) config: nothing drives. It used to mean on, with the fixed presets (3-4 mph under)
  return value if value in ('on', 'off') else 'off'


def pitch_zero(root: Path) -> float:
  """The constant nose-up reading of the pitch signal, as the judge last measured it (experiments.json)."""
  try:
    zero = json.loads((root / 'experiments.json').read_text())['corrections']['pitch_zero']
    return float(zero) if isinstance(zero, (int, float)) and abs(zero) < 0.1 else 0.0
  except (OSError, ValueError, TypeError, KeyError, AttributeError):
    return 0.0


def stop_distance(force_stop_m: float | None, lead) -> float | None:
  """Metres to the nearer stop: Force Stop's line, or STOP_GAP_M behind a stopped car ahead; None when neither."""
  options = [force_stop_m] if isinstance(force_stop_m, (int, float)) and math.isfinite(force_stop_m) and force_stop_m > 0 else []
  if lead.present and lead.vLead < 1.0 and math.isfinite(lead.dRel):
    options.append(max(0.0, float(lead.dRel) - STOP_GAP_M))
  return min(options) if options else None


class ExperimentHost:
  def __init__(self, root: Path | None = None, status_path: Path = STATUS_PATH, clock=time.monotonic):
    self.root = root
    self.status_path = status_path
    self.clock = clock
    self.drive = None
    self.enabled = False
    self.exp = Experimenter()
    self.shadow: DialExperimenter | None = None   # engine 'shadow': the dials decide and log, the fixed variants drive
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
    self.root_now = root
    from openpilot.starpilot.efficiency.dials import load_state
    from openpilot.starpilot.efficiency.experiment_learn import engine
    which = engine(root)
    vetoes = load_vetoes(root / 'vetoes.json')
    dials = load_state(root / 'experiments.json')
    if which == 'dials':
      self.exp = DialExperimenter(vetoes=vetoes, state=dials)
      self.shadow = None
    else:
      self.exp = Experimenter(winners=load_winners(root / 'experiments.json'), vetoes=vetoes)
      self.shadow = DialExperimenter(vetoes=dict(vetoes), state=dials) if which == 'shadow' else None
    carried = load_budget(root / BUDGET_FILE)
    for exp in (self.exp, self.shadow):
      if exp is not None and carried is not None:
        exp.carry_engaged_s, exp.carry_spent_s = carried
        exp.grace_s = 0.0   # only the very first drive has nothing earned yet
    self.pitch_zero = pitch_zero(root)
    self.state_mtime = self._state_mtime()
    self.next_reload = 0.0
    self.shadow_tag: str | None = None
    self.saved_vetoes = dict(self.exp.vetoes)
    self.driver_pulse_glide = False
    self.we_turned_pulse_glide_on = False
    from openpilot.starpilot.diagnostics.features import ChangeLog
    self.log = ChangeLog('efficiency_experiment', min_gap_s=1.0)

  def _state_mtime(self) -> float | None:
    try:
      return (self.root_now / 'experiments.json').stat().st_mtime
    except OSError:
      return None

  def _reload(self, now: float) -> None:
    """The judge wrote a new experiments.json (it runs as the drive starts): take its dial centers and pitch zero."""
    if now < self.next_reload:
      return
    self.next_reload = now + RELOAD_CHECK_S
    mtime = self._state_mtime()
    if mtime == self.state_mtime:
      return
    self.state_mtime = mtime
    from openpilot.starpilot.efficiency.dials import load_state
    state = load_state(self.root_now / 'experiments.json')
    for exp in (self.exp, self.shadow):
      if isinstance(exp, DialExperimenter):
        exp.state = dict(state)
    self.pitch_zero = pitch_zero(self.root_now)

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

  def sample(self, sm, wheel_coast, *, drive, force_stop_forcing: bool, stop_distance_m: float | None = None) -> Plan | None:
    """The plan for this cycle, or None when experiments are off or cannot run."""
    try:
      if drive != self.drive:
        self._new_drive(drive)
      if not self.enabled:
        return None
      now = self.clock()
      self._reload(now)
      self._read_soc(now)
      owner = getattr(wheel_coast, 'owner', None)
      if owner is not None and getattr(owner, 'change_reason', '') == DRIVER_PULSE_GLIDE:
        self.driver_pulse_glide = True   # the driver chose: Pulse and Glide stays theirs for the rest of the drive
      cs, cc = sm['carState'], sm['carControl']
      lead = sm['radarState'].leadOne
      v_ego = float(cs.vEgo)
      gap = float(lead.dRel) / max(v_ego, 1.0) if lead.present else math.inf
      pitch = cc.orientationNED[1] if len(cc.orientationNED) == 3 else None
      grade = math.tan(pitch - self.pitch_zero) if pitch is not None and math.isfinite(pitch) else None
      stop_ahead = bool(force_stop_forcing or sm['modelV2'].action.shouldStop or sm['controlsState'].forceDecel or
                        (lead.present and lead.vLead < 1.0 and lead.dRel < 60.0))
      frame = Frame(now, v_ego, float(cs.vCruise) * KPH_TO_MS, bool(cc.longActive and cc.enabled), bool(cs.gasPressed),
                    bool(cs.brakePressed), bool(cs.standstill), grade, gap, stop_ahead, self.soc, self.driver_pulse_glide,
                    float(cs.aEgo), bool(cs.leftBlinker or cs.rightBlinker),
                    stop_distance_m=stop_distance(stop_distance_m, lead))
      plan = self.exp.sample(frame)
      if self.shadow is not None:
        shadow_plan = self.shadow.sample(frame)   # decides and logs only; its vetoes and time are never saved
        if shadow_plan.tag != self.shadow_tag:
          self.shadow_tag = shadow_plan.tag
          from openpilot.starpilot.diagnostics.features import EVENT
          from openpilot.common.swaglog import cloudlog
          cloudlog.event(EVENT, feature='efficiency_dials_shadow', tag=shadow_plan.tag, driving=plan.tag,
                         budget_left_s=round(self.shadow.budget_left_s(), 1))
      if self.exp.vetoes != self.saved_vetoes:   # the driver pressed on during a slower variant: remember it
        # merge with the file (it may have gained entries since the drive began), keeping the higher count
        path = self.root_now / 'vetoes.json'
        merged = load_vetoes(path)
        for key, count in self.exp.vetoes.items():
          merged[key] = max(merged.get(key, 0), count)
        self.exp.vetoes = dict(merged)
        self.saved_vetoes = dict(merged)
        save_vetoes(path, merged)
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
    if owner is None:
      return
    if self.driver_pulse_glide:
      if getattr(owner, 'tuning', None) is not None:
        owner.tuning = None   # the driver took Pulse and Glide over: their band and the normal pulse and glide
      return
    if plan.pulse_glide is True and not owner.pulse_glide:
      owner.pulse_glide = True
      owner.change_reason = 'efficiency experiment'
      self.we_turned_pulse_glide_on = True
    if plan.pulse_glide is True and self.we_turned_pulse_glide_on:
      from openpilot.starpilot.controllers.coast import PulseGlideTuning
      tuning = plan.pulse_glide_tuning
      owner.tuning = PulseGlideTuning(*tuning) if tuning is not None else None
    elif hasattr(owner, 'tuning'):
      owner.tuning = None   # the driver's own Pulse and Glide, or none: their band, the normal pulse and glide
    if plan.pulse_glide is not True and self.we_turned_pulse_glide_on:
      # the experiment ended: hand back what the driver had (off) unless they took it over meanwhile
      if owner.pulse_glide:
        owner.pulse_glide = False
        owner.change_reason = 'efficiency experiment ended'
      self.we_turned_pulse_glide_on = False
