"""Driving health requirements for manager-owned services."""

# Projection and local phone access are optional even when requested.
# Their lifecycles remain visible in managerState and their own status.
# A dead Jetlink owner sends the drive back to the small model.
# The energy ledger and the BlueDriver logger only observe; their absence must never block or disengage driving.
ANCILLARY_PROCESSES = frozenset({'starportd', 'galaxy_hotspot', 'jetlinkd', 'efficiencyd', 'edgemapd', 'bluedriverd', 'curvemapd', 'sideroadd', 'traced'})


def driving_process_failures(processes) -> set[str]:
  return {process.name for process in processes
          if process.shouldBeRunning and not process.running and process.name not in ANCILLARY_PROCESSES}
