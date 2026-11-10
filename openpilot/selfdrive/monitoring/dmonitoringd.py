#!/usr/bin/env python3
import os
import time
import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process
from openpilot.selfdrive.monitoring.policy import DriverMonitoring
from openpilot.starpilot.aol.runtime import monitoring_lateral_engaged
from openpilot.starpilot.feature_runtime import enabled as feature_enabled, requested as feature_requested

AOL_SERVICES = ['aolAxisState', 'aolSafetyWire']


def aol_runtime_armed(params) -> bool | None:
  """None until card publishes this drive's CarParams (cleared at every onroad transition),
  then whether the drive started AOL-armed. Some cars arm AOL even with the toggle off so the
  toggle works live; monitoring must follow the armed runtime, not the startup toggle."""
  raw = params.get("CarParams")
  if not raw:
    return None
  from opendbc.car.structs import car
  return feature_enabled(params, messaging.log_from_bytes(raw, car.CarParams), 'aol', os.environ)


def dmonitoringd_thread():
  config_realtime_process([0, 1, 2, 3], 5)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState'])
  aol_replay = os.getenv('AOL_REPLAY_RUNTIME') == '1' or feature_requested(params, 'aol')
  required = ['driverStateV2', 'extrinsicsCalibration', 'carState', 'selfdriveState', 'modelV2']
  sm = messaging.SubMaster(required + (AOL_SERVICES if aol_replay else []), poll='driverStateV2')
  aol_arming_pending = not aol_replay

  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))
  demo_mode=False

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      # iterate when model has new output
      continue

    if aol_arming_pending and sm['driverStateV2'].frameId % 20 == 2:
      armed = aol_runtime_armed(params)
      if armed is not None:
        aol_arming_pending = False
        if armed:
          aol_replay = True
          sm = messaging.SubMaster(required + AOL_SERVICES, poll='driverStateV2')
          continue

    valid = sm.all_checks(required)
    aol_engaged = aol_replay and monitoring_lateral_engaged(sm, now_ns=time.monotonic_ns())
    if demo_mode and sm.valid['driverStateV2']:
      DM.run_step(sm, demo=True)
    elif valid:
      DM.run_step(sm, demo=demo_mode, aol_lateral_engaged=aol_engaged)

    # publish
    dat = DM.get_state_packet(valid=valid)
    pm.send('driverMonitoringState', dat)

    # load live always-on toggle
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")
      demo_mode = params.get_bool("IsDriverViewEnabled")

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and not demo_mode and
     DM.wheelpos_offsetter.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_offsetter.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool("IsRhdDetected", DM.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
