from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigMultiToggle, BigToggle
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.starpilot.ui import l4_custom as settings


def _fmt_delta(value: float, metric: bool) -> str:
  return f"{value:g} {'km/h' if metric else 'mph'}"


def _fmt_offset(value: float) -> str:
  return f"{value:+.2f} m"


class L4CustomLayoutMici(NavScroller):
  """One tile per L4 setting. Personal steering and curve guard apply from the next drive; the rest apply live."""

  def __init__(self):
    super().__init__()
    params = ui_state.params

    self._nnlc = BigMultiToggle("personal steering", list(settings.NNLC_MODES), select_callback=self._on_nnlc,
                                description="Your own steering model, trained on your drives.\n" +
                                            "auto: the trainer's on-road trial decides.\n" +
                                            "off: always standard steering. on: always your model.\n" +
                                            "Applies from the next drive.")
    self._curve_guard = BigToggle("curve guard", initial_state=settings.curve_guard_enabled(), toggle_callback=self._on_curve_guard,
                                  description="In curves, keeps the car's inside body off a clearly seen inside lane line or road edge.\n" +
                                              "Off with the turn signal and below 18 mph. Applies from the next drive.")
    self._metric = settings.is_metric(params)
    self._pg_options = settings.pg_delta_options(params)
    self._pg = BigMultiToggle("pulse & glide band", [_fmt_delta(v, self._metric) for v in self._pg_options], select_callback=self._on_pg,
                              description="How far below the set speed each glide coasts before the next pulse.")
    self._cem = BigToggle("conditional experimental", initial_state=settings.cem_on(params), toggle_callback=self._on_cem,
                          description="Switches to experimental mode only when needed (stops, lights, slow leads).\n" +
                                      "Off uses the standard mode. Change while not engaged.")
    self._cem.set_enabled(lambda: not ui_state.engaged)
    self._offset_options = settings.lane_offset_options(params)
    self._offset = BigMultiToggle("lane offset", [_fmt_offset(v) for v in self._offset_options], select_callback=self._on_offset,
                                  description="Where the comma sits relative to the car's center (negative: left of center).\n" +
                                              "-0.23 m matches a comma mounted 9 in left. Also used by the curve guard.")

    self._scroller.add_widgets([self._nnlc, self._curve_guard, self._pg, self._cem, self._offset])

  def show_event(self):
    super().show_event()
    self._refresh()

  def _refresh(self):
    params = ui_state.params
    self._nnlc.set_value(settings.nnlc_override())
    self._curve_guard.set_checked(settings.curve_guard_enabled())
    self._cem.set_checked(settings.cem_on(params))
    pg = settings.pg_delta(params)
    self._pg.set_value(_fmt_delta(min(self._pg_options, key=lambda v: abs(v - pg)), self._metric))
    offset = settings.lane_offset(params)
    self._offset.set_value(_fmt_offset(min(self._offset_options, key=lambda v: abs(v - offset))))

  def _on_nnlc(self, value: str):
    settings.set_nnlc_override(value)

  def _on_curve_guard(self, enabled: bool):
    settings.set_curve_guard(enabled)

  def _on_pg(self, label: str):
    settings.set_pg_delta(ui_state.params, self._pg_options[[_fmt_delta(v, self._metric) for v in self._pg_options].index(label)])

  def _on_cem(self, on: bool):
    if not settings.set_cem(ui_state.params, on, authorized=lambda: not ui_state.engaged):
      self._cem.set_checked(settings.cem_on(ui_state.params))

  def _on_offset(self, label: str):
    settings.set_lane_offset(ui_state.params, self._offset_options[[_fmt_offset(v) for v in self._offset_options].index(label)])
