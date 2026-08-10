# RevPi A external system-event logging

This deployment adds durable robot/feeder/hardware errors to the existing
`/events/today` and `/events/date` JSON APIs served by `web_hp.py`.

It is intentionally isolated from Fill HP's existing data:

- Existing file, unchanged: `/home/pi/cartridge_fill_logs/system_stop_events.csv`
- New file: `/home/pi/cartridge_fill_logs/external_system_events.csv`
- Existing API response shape remains unchanged. Rows from both files are
  merged and the `count`, `error`, and `warn` totals are recalculated.

## Events captured

The bridge records a row when a source **transitions into** an error state,
but only while the synchronized system mode is **AUTO** or **AI**:

- `/robot/error`: any non-empty/non-healthy error message
- `/robot/system_status`: `ERROR_*`, fault, failure, timeout, offline, or
  disconnected status
- `/providesystem/gui_notify`: JSON messages whose `level` is `error`
- `/system_state`: feeder `ERROR`/fault status
- Critical Fill HP `hw_status` (`error_status` stays exclusively owned by the
  existing `system_stop_events.csv` logger, preventing duplicate rows)
- Loadcell critical status/calibration status, overload, and zero-drift warning
- `/vfd/status`: critical VFD connection/fault transition
- `/camera/status`, `/camera/cam0/health`, `/camera/cam1/health`, and
  `/vision/roi_status`: reconnect warning or error/fatal/failure/timeout/
  offline/CSI/ROI validation transition

`/weight/monitor_status` values such as `LAST:FAIL` and the related
`/loadcell/consecutive_fails` counter are intentionally excluded: they describe
normal NG cartridges routed to the fail position, not a stopped system. Camera
health counters are normalized before de-duplication so changing FPS/age values
cannot create one row per second during a reconnect incident.

Only `ERROR` rows belong to Stop Errors. `WARN` events and every fault observed
in MANUAL/JOG/unknown mode remain runtime diagnostics and are not written to
either production Stop Errors CSV. The merged API also filters legacy
Manual/Warning rows without deleting the original CSV history.

Repeated samples of one active fault are suppressed. After a healthy/recovered
sample, recurrence is logged as a new event. A ten-second persistent de-duplication
window also prevents a latched topic from being duplicated immediately after
`web_hp` restarts.

Logging is fail-safe: a CSV or parsing error is reported to the ROS logger but
is not allowed to escape into a ROS callback.

## Safe deployment

The deploy command is **read-only by default**:

```bash
cd /home/nhan/ros2_ws
scripts/revpi_event_logging/deploy_revpi_event_logging.sh
```

Review its target and then apply explicitly:

```bash
scripts/revpi_event_logging/deploy_revpi_event_logging.sh --apply
```

The apply operation:

1. stages the tracked logger and patcher in a unique `/tmp` directory on RevPi A;
2. parses and compiles the complete proposed `web_hp.py` before writing;
3. patches `production_log.py` so Fill HP follows the same AUTO/AI + ERROR-only
   persistence policy;
4. installs the companion module in the source package, then runs
   `colcon build --packages-select fill_hp --symlink-install`;
5. verifies `from fill_hp import external_event_log` through the workspace's
   `install/setup.bash` before changing `web_hp.py`;
6. stores timestamped backups in
   `/home/pi/cartridge_fill_logs/web_hp_backups`;
7. atomically replaces `web_hp.py` and verifies all changed files;
8. removes the staging directory.

It builds only the `fill_hp` package so the new Python import is valid. It does
**not** restart, stop, or reload any service. The new callbacks become active
only after `web_hp` is restarted separately in a controlled maintenance window.
The build supports both an apt-installed `/opt/ros/jazzy` setup and this RevPi's
source-built Jazzy setup in the workspace's existing `install/setup.bash`.

The current known source target is:

```text
/home/pi/ros2_jazzy/src/fill_hp/fill_hp/web_hp.py
```

Override the host or target only if RevPi deployment layout changes:

```bash
scripts/revpi_event_logging/deploy_revpi_event_logging.sh \
  --host pi@172.16.11.31 \
  --target /absolute/path/to/web_hp.py \
  --apply
```

## Rollback

No automatic rollback or restart is performed. To disable the feature during a
maintenance window, restore the timestamped `web_hp.py.*.bak`, verify it, and
then restart only `web_hp`. On the first installation the companion module is a
new file, so it has no backup; leaving that unused file in place is harmless.
Remove it and rebuild `fill_hp` only when a complete cleanup is explicitly
required.

## Tests

The tests use temporary files and a synthetic `web_hp.py`; they never connect to
RevPi A:

```bash
python3 -m pytest -q scripts/revpi_event_logging/tests
```
