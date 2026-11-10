"""Wheel-energy model for the hybrid efficiency ledger (observe only; it never commands the car).

Per unit mass, the force the powertrain must supply at the wheels is
  a + a_resist(v) + g * sin(pitch)
where a_resist is the natural coast deceleration (rolling + driveline + aero) already calibrated for
Pulse and Glide on the Elantra Hybrid. Positive power is propulsion. Negative power must be absorbed:
by regen up to its limits, otherwise by the friction brakes (lost as heat).

The regen limits are estimates for a compact full hybrid; they only split braking energy into
"recoverable" and "friction" and can be refined once the hybrid battery signal is calibrated.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.starpilot.controllers.coast import G, GLIDE_C0, GLIDE_C2

REGEN_MAX_POWER_W = 30_000.0  # estimate: compact full-hybrid motor/generator
REGEN_MAX_DECEL = 2.0  # m/s^2 beyond natural coast that regen can supply; harder stops use friction too
REGEN_MIN_SPEED = 2.0  # m/s; regen fades out at crawl speeds, the last metres stop on friction


def resist_accel(speed_mps: float) -> float:
  """Natural coast deceleration on a flat road (positive number, m/s^2)."""
  return GLIDE_C0 + GLIDE_C2 * speed_mps ** 2


def grade_accel(pitch_rad: float | None) -> float:
  return G * math.sin(pitch_rad) if pitch_rad is not None and math.isfinite(pitch_rad) else 0.0


@dataclass(frozen=True)
class EnergyStep:
  propulsion_j: float  # energy the powertrain delivered to the wheels
  regen_j: float  # braking energy within estimated regen capability
  friction_j: float  # braking energy beyond it (heat)
  steady_j: float  # energy steady cruising at this speed and grade would have needed


def step(speed_mps: float, accel_mps2: float, pitch_rad: float | None, mass_kg: float, dt_s: float) -> EnergyStep:
  """Energy for one sample interval. Reverse/standstill intervals contribute nothing."""
  if not (math.isfinite(speed_mps) and math.isfinite(accel_mps2) and math.isfinite(dt_s)) or speed_mps <= 0.05 or dt_s <= 0:
    return EnergyStep(0.0, 0.0, 0.0, 0.0)
  grade = grade_accel(pitch_rad)
  demand = accel_mps2 + resist_accel(speed_mps) + grade  # m/s^2 the powertrain/brakes must supply
  power = mass_kg * speed_mps * demand
  steady = mass_kg * speed_mps * max(0.0, resist_accel(speed_mps) + grade)
  if power >= 0.0:
    return EnergyStep(power * dt_s, 0.0, 0.0, steady * dt_s)
  absorbed = -power * dt_s
  if speed_mps < REGEN_MIN_SPEED:
    return EnergyStep(0.0, 0.0, absorbed, steady * dt_s)
  regen_power = min(-power, REGEN_MAX_POWER_W, mass_kg * speed_mps * REGEN_MAX_DECEL)
  regen = regen_power * dt_s
  return EnergyStep(0.0, regen, absorbed - regen, steady * dt_s)
