import math
from dataclasses import dataclass

G = 9.81
# Pulse and Glide. The glide commands the car's natural coasting deceleration so the
# powertrain makes ~zero torque (no engine, no regen); commanding less deceleration than
# that makes the powertrain push, which at highway speed keeps the engine running.
# Natural coast ~= -(c0 + c2 * v^2) on a flat road: rolling + driveline loss plus aero.
# Calibrated against Elantra Hybrid lift-off coasting (-0.19 m/s^2 at 35 mph, -0.27 at 65)
# and generic passenger-car physics.
GLIDE_C0 = 0.12
GLIDE_C2 = 0.00018
GLIDE_FLOOR_MARGIN = 0.05
# EV feather glide. Elantra Hybrid logs (0x595 power flow vs the dash EV light): below ~63 mph a glide
# that just cancels regen (~-0.05 m/s^2 on a flat road) stays electric with the battery about neutral
# (EV 97% at 60-62 mph); above ~64 mph the same light request starts the engine (EV 12% at 64-66 mph),
# so the regen glide above is kept there. The cut-off is a starting point for the efficiency learner.
FEATHER_ACCEL = -0.05
FEATHER_MAX_SPEED_MPS = 63.0 * 0.44704
# Ease between pulse and glide instead of stepping: a hard step from +0.5 to -0.45 m/s^2 showed a
# regen spike with the engine still on (mode 128, power meter pinned) at every glide start.
TRANSITION_JERK = 0.6  # m/s^3
DT = 0.05
# A glide never carries the car past the set speed (downhill): hold it at the set speed instead.
GLIDE_SET_SPEED_GAIN = 1.0
# The pulse is a steady, moderate acceleration through the band (engine at an efficient load),
# easing only just below the set speed so it never overshoots.
PULSE_ACCEL_BP = [10.0, 20.0, 30.0]
PULSE_ACCEL_V = [0.8, 0.6, 0.45]
PULSE_ARRIVAL_GAIN = 1.5
# Far below the band (after slowing for traffic), ordinary cruise owns the return.
PULSE_ENTRY_MARGIN = 1.0
# The band is the full set speed down to set - delta: the pulse runs until the set speed is reached
# (less a hair, since its arrival easing approaches the set speed asymptotically), and the glide runs
# all the way to the bottom.
GLIDE_START_MARGIN = 0.1
GLIDE_END_MARGIN = 0.0
# Switch early by the speed the car still gains or loses after a switch: easing the command at
# TRANSITION_JERK carries it a^2 / (2 * jerk) further, and a pulse also waits on the hybrid to start
# pushing (often an engine start). A 40 mph drive measured 0.39 mph past the glide->pulse switch
# (accel -0.3) and 0.75 mph past the pulse->glide switch (accel +0.62), landing on dash 34 and 41.
PULSE_START_LAG_S = 0.35
# The band follows the dash speedometer (what the driver set and sees), not the slightly lower wheel
# speed. Plausible dash/wheel ratios only; anything else falls back to wheel speed.
CLUSTER_RATIO_MIN = 0.95
CLUSTER_RATIO_MAX = 1.10
CLUSTER_RATIO_MIN_SPEED = 5.0
# The dash speed is whole mph, so one sample's ratio jumps about +/-2.5% at 40 mph (the band edges moved
# +/-0.75 mph). A slow average settles on the true ratio while the speed sweeps the band.
CLUSTER_RATIO_TAU_S = 30.0


def glide_accel(speed_mps, pitch):
  """Glide target (actual acceleration): an EV feather below the cut-off, the regen glide above it."""
  grade = G * math.sin(pitch) if type(pitch) in (int, float) and math.isfinite(pitch) else 0.0
  if speed_mps < FEATHER_MAX_SPEED_MPS:
    return FEATHER_ACCEL - grade
  return regen_glide_accel(speed_mps) - grade


def regen_glide_accel(speed_mps):
  """Natural coasting on a flat road (rolling + driveline loss plus aero); light regen on the Elantra."""
  return -(GLIDE_C0 + GLIDE_C2 * speed_mps ** 2)


def raw_cluster_ratio(car_state) -> float | None:
  """One sample of dash speedometer / wheel speed, or None when it is not usable."""
  try:
    v, dash = float(car_state.vEgo), float(car_state.vEgoCluster)
  except (AttributeError, TypeError, ValueError):
    return None
  if not (math.isfinite(v) and math.isfinite(dash)) or v < CLUSTER_RATIO_MIN_SPEED:
    return None
  ratio = dash / v
  return ratio if CLUSTER_RATIO_MIN <= ratio <= CLUSTER_RATIO_MAX else None


def cluster_ratio(car_state) -> float:
  """Dash speedometer / wheel speed (Hyundai's dash reads about 2.7% high); 1.0 when unknown."""
  ratio = raw_cluster_ratio(car_state)
  return 1.0 if ratio is None else ratio


def carry_mps(accel, lag_s=0.0):
  """Speed the car still gains (accel > 0) or loses after the command starts easing toward the other phase."""
  if type(accel) not in (int, float) or not math.isfinite(accel):
    return 0.0
  return accel * accel / (2.0 * TRANSITION_JERK) + abs(accel) * lag_s


class ClusterRatioFilter:
  """A slow average of dash/wheel speed; 1.0 until the first plausible sample."""
  def __init__(self, tau_s=CLUSTER_RATIO_TAU_S, dt=DT):
    self.alpha = dt / (tau_s + dt)
    self.value = None

  def update(self, car_state) -> float:
    ratio = raw_cluster_ratio(car_state)
    if ratio is not None:
      self.value = ratio if self.value is None else self.value + self.alpha * (ratio - self.value)
    return 1.0 if self.value is None else self.value


def pulse_accel(speed_mps):
  bp, values = PULSE_ACCEL_BP, PULSE_ACCEL_V
  if speed_mps <= bp[0]:
    return values[0]
  for i in range(1, len(bp)):
    if speed_mps <= bp[i]:
      return values[i - 1] + (values[i] - values[i - 1]) * (speed_mps - bp[i - 1]) / (bp[i] - bp[i - 1])
  return values[-1]


@dataclass(frozen=True)
class CoastPlan:
  force_decel: bool = False
  brake_floor: float | None = None
  speed_ceiling_mps: float | None = None
  pulse_coasting: bool = False
  # Cruise acceleration target while Pulse and Glide owns the cruise speed (pulse or glide).
  accel_target: float | None = None
  pulsing: bool = False
  # Logging only: the band in wheel speed while Pulse and Glide is enabled, and the hill pause.
  band_top_mps: float | None = None
  band_bottom_mps: float | None = None
  hill_paused: bool = False


class CoastOwner:
  def __init__(self):
    self.force_coast = False
    self.pulse_glide = False
    self.coasting = False
    self.hill_paused = False
    self.last_target = None

  def reset(self):
    self.__init__()

  def toggle(self, action, *, long_active):
    if action == 2:
      self.force_coast = not self.force_coast
      return True
    if action == 14 and (long_active or self.pulse_glide):
      self.pulse_glide = not self.pulse_glide
      return True
    return False

  def sample(self, *, admitted, active, gas, brake, speed_mps, target_mps, delta_mps, pitch,
             lead_relevant, stop_context):
    values = (speed_mps, target_mps, delta_mps)
    if (not admitted or not all(type(value) in (int, float) and math.isfinite(value) for value in values) or
        speed_mps < 0 or delta_mps < 0 or type(active) is not bool or type(gas) is not bool or
        type(brake) is not bool or type(lead_relevant) is not bool or type(stop_context) is not bool):
      self.reset()
      return CoastPlan()
    self.force_coast &= not (gas or brake)
    if not active:
      self.last_target = None
      return CoastPlan()
    accel_target = None
    pulsing = False
    band = (None, None)
    if not self.pulse_glide:
      self.coasting = False
      self.hill_paused = False
    else:
      if type(pitch) in (int, float) and math.isfinite(pitch):
        if self.hill_paused:
          if abs(pitch) <= math.radians(2.5):
            self.hill_paused = False
        elif abs(pitch) >= math.radians(3.0):
          self.hill_paused = True
      lower = target_mps - delta_mps
      band = (target_mps, lower)
      available = not (self.hill_paused or target_mps <= 5.0 or lower < 3.0 or delta_mps <= 0 or
                       lead_relevant or stop_context)
      if not available:
        self.coasting = False
      elif self.coasting:
        slowing = min(self.last_target, 0.0) if self.last_target is not None else 0.0
        if speed_mps - carry_mps(slowing, PULSE_START_LAG_S) <= lower + GLIDE_END_MARGIN:
          self.coasting = False
      else:
        gaining = max(self.last_target, 0.0) if self.last_target is not None else 0.0
        if speed_mps + carry_mps(gaining) >= target_mps - GLIDE_START_MARGIN:
          self.coasting = True
      if available and self.coasting:
        accel_target = min(glide_accel(speed_mps, pitch), GLIDE_SET_SPEED_GAIN * (target_mps - speed_mps))
      elif available and speed_mps >= lower - PULSE_ENTRY_MARGIN:
        pulsing = True
        accel_target = min(pulse_accel(speed_mps), PULSE_ARRIVAL_GAIN * max(0.0, target_mps - speed_mps))
      if accel_target is not None and self.last_target is not None:
        step = TRANSITION_JERK * DT
        accel_target = min(max(accel_target, self.last_target - step), self.last_target + step)
    self.last_target = accel_target
    if self.force_coast:
      return CoastPlan(True, -0.6, None, self.coasting, band_top_mps=band[0], band_bottom_mps=band[1],
                       hill_paused=self.hill_paused)
    brake_floor = accel_target - GLIDE_FLOOR_MARGIN if self.coasting and accel_target is not None else None
    return CoastPlan(False, brake_floor, None, self.coasting, accel_target, pulsing, band[0], band[1], self.hill_paused)


class CoastRuntime:
  def __init__(self):
    from openpilot.starpilot.controllers.wheel_actions import WheelConsumer
    self.consumer = WheelConsumer()
    self.owner = CoastOwner()
    self.session = None
    self.drive = 0
    self.traffic_toggles = 0
    self.cp_signature = None
    self.conditional_stop = False
    self.cluster_ratio = 1.0  # logging: the dash/wheel ratio used by the last sample
    self.ratio_filter = ClusterRatioFilter()

  def receive(self, event, params, cp, sm, *, now_ns):
    command = self.consumer.accept(event, params, cp, sm, now_ns=now_ns)
    if command is None:
      return False
    if command.session != self.session or command.drive_id != self.drive:
      self.owner.reset()
      self.session, self.drive = command.session, command.drive_id
    from openpilot.starpilot.controllers.wheel_actions import cp_fingerprint
    self.cp_signature = cp_fingerprint(cp)
    if command.action in (2, 6, 14):
      from openpilot.starpilot.conditional_mode.ui_action import current_authority
      from openpilot.starpilot.saved_source import read_saved
      active_required = command.action == 6 or command.action == 14 and not self.owner.pulse_glide
      if (not cp.openpilotLongitudinalControl or
          read_saved(params, "SafeMode", 8) not in ((None, True), (b"0", True)) or
          active_required and not current_authority(sm, cp, command.drive_id, now_ns)):
        return False
      if command.action == 6:
        self.traffic_toggles += 1
        return True
      return self.owner.toggle(command.action, long_active=bool(sm["carControl"].longActive))
    return False

  def sample(self, params, cp, sm, *, now_ns, target_mps, lead_relevant, stop_context):
    from openpilot.starpilot.controllers.wheel_actions import eligible, LIFETIME_NS, cp_fingerprint, WheelConsumer
    from openpilot.starpilot.conditional_mode.ui_action import fresh_service
    from openpilot.starpilot.saved_source import read_saved
    # A zero accepted-drive/heartbeat cannot satisfy the admission below.
    if (type(self) is CoastRuntime and type(getattr(self, 'owner', None)) is CoastOwner and
        type(getattr(self, 'consumer', None)) is WheelConsumer and 'sample' not in vars(self.owner) and
        type(getattr(self, 'drive', None)) is int and type(getattr(self.consumer, 'last_heartbeat', None)) is int and
        (self.drive == 0 or self.consumer.last_heartbeat == 0) and
        type(stop_context) is bool and type(getattr(self, 'conditional_stop', None)) is bool):
      from capnp.lib.capnp import _DynamicStructReader
      from opendbc.car.structs import car
      from openpilot.cereal import log, messaging
      from openpilot.common.params import Params
      if (type(params) is Params and type(sm) is messaging.SubMaster and type(getattr(sm, 'data', None)) is dict and
          type(cp) is _DynamicStructReader and cp.schema == car.CarParams.schema):
        device = sm.data.get('deviceState')
        state = sm.data.get('carState')
        control = sm.data.get('carControl')
        if (type(device) is _DynamicStructReader and device.schema == log.DeviceState.schema and
            type(state) is _DynamicStructReader and state.schema == car.CarState.schema and
            type(control) is _DynamicStructReader and control.schema == car.CarControl.schema):
          self.owner.reset()
          return CoastPlan()
    drive = int(sm["deviceState"].startedMonoTime)
    raw, readable = read_saved(params, "PulseGlideSpeedDelta", 32)
    units, unit_readable = read_saved(params, "IsMetric", 8)
    try:
      delta = 5.0 if raw is None else float(raw)
    except (ValueError, TypeError, OverflowError):
      delta = math.nan
    admitted = bool(eligible(cp) and self.cp_signature == cp_fingerprint(cp) and cp.openpilotLongitudinalControl and sm["deviceState"].started and
                    drive == self.drive and 0 < drive < self.consumer.last_heartbeat <= now_ns and
                    now_ns - self.consumer.last_heartbeat <= LIFETIME_NS and
                    all(fresh_service(sm, name, drive, now_ns) for name in ("carState", "carControl")) and
                    sm["carState"].canValid and not sm["carState"].canTimeout and
                    read_saved(params, "SafeMode", 8) in ((None, True), (b"0", True)) and
                    readable and unit_readable and units in (None, b"0", b"1") and
                    math.isfinite(delta) and 0.5 <= delta <= 30.0)
    state, control = sm["carState"], sm["carControl"]
    pitch = control.orientationNED[1] if len(control.orientationNED) == 3 else None
    # Express the dash-speed band in wheel speed: the set speed and delta the driver sees on the dash.
    ratio_filter = getattr(self, 'ratio_filter', None)
    ratio = ratio_filter.update(state) if ratio_filter is not None else cluster_ratio(state)
    self.cluster_ratio = ratio
    return self.owner.sample(admitted=admitted, active=bool(control.enabled and control.longActive),
                             gas=bool(state.gasPressed), brake=bool(state.brakePressed), speed_mps=float(state.vEgo),
                             target_mps=target_mps / ratio,
                             delta_mps=delta * (1 / 3.6 if units == b"1" else 0.44704) / ratio,
                             pitch=pitch, lead_relevant=lead_relevant, stop_context=bool(stop_context or self.conditional_stop))


def plan_log_fields(plan: CoastPlan | None, runtime=None) -> dict:
  """Pulse and Glide fields for starpilotLongitudinalPlan (all off when the mode is off)."""
  enabled = bool(runtime is not None and getattr(getattr(runtime, 'owner', None), 'pulse_glide', False))
  if plan is None:
    return {'pulseGlideEnabled': enabled, 'pulseGlidePhase': 'off'}
  phase = ('forceCoast' if plan.force_decel else 'glide' if plan.pulse_coasting and plan.accel_target is not None else
           'pulse' if plan.pulsing else 'off')
  finite = lambda v: float(v) if isinstance(v, (int, float)) and math.isfinite(v) else 0.0  # noqa: E731
  return {'pulseGlideEnabled': enabled, 'pulseGlidePhase': phase,
          'pulseGlideAccelTarget': finite(plan.accel_target), 'pulseGlideBandTop': finite(plan.band_top_mps),
          'pulseGlideBandBottom': finite(plan.band_bottom_mps),
          'pulseGlideClusterRatio': finite(getattr(runtime, 'cluster_ratio', 1.0)),
          'pulseGlideHillPaused': bool(plan.hill_paused)}
