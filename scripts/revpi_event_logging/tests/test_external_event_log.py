import csv
from datetime import datetime, timedelta, timezone

import pytest

from external_event_log import EVENT_HEADER, ExternalEventLog, merge_event_summaries


class Clock:
    def __init__(self):
        self.value = datetime(2026, 8, 5, 15, 0, 0, tzinfo=timezone(timedelta(hours=7)))

    def now(self):
        return self.value

    def advance(self, seconds):
        self.value += timedelta(seconds=seconds)


def read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def test_robot_error_repeats_once_and_healthy_status_starts_new_incident(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)

    assert log.observe("robot_error", "ERROR_PICK_TIMEOUT")
    for _ in range(100):
        clock.advance(0.1)
        assert not log.observe("robot_error", "ERROR_PICK_TIMEOUT")
    assert len(read_rows(path)) == 1

    # /robot/error is an event stream and need not emit an empty message. A
    # healthy /robot/system_status explicitly closes the incident.
    assert not log.observe("robot_system_status", "STANDBY")
    assert log.observe("robot_error", "ERROR_PICK_TIMEOUT")
    assert len(read_rows(path)) == 2


def test_quiet_gap_distinguishes_recurrence_without_healthy_sample(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(
        path=str(path), now_fn=clock.now, incident_quiet_seconds=30
    )

    assert log.observe("robot_error", "ERROR_NO_TRAY")
    clock.advance(2)
    assert not log.observe("robot_error", "ERROR_NO_TRAY")
    clock.advance(31)
    assert log.observe("robot_error", "ERROR_NO_TRAY")
    assert len(read_rows(path)) == 2


def test_failed_write_does_not_latch_and_lose_the_incident(tmp_path, monkeypatch):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)
    real_ensure = log._ensure_file

    monkeypatch.setattr(log, "_ensure_file", lambda: False)
    assert not log.observe("robot_error", "DOBOT_CONTINUE_FAILED")

    monkeypatch.setattr(log, "_ensure_file", real_ensure)
    assert log.observe("robot_error", "DOBOT_CONTINUE_FAILED")
    assert len(read_rows(path)) == 1


def test_feeder_error_notification_is_transition_aware_and_state_recovers_it(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)
    payload = '{"level":"error","title":"Servo offline","detail":"InX timeout"}'

    assert log.observe("feeder_gui_notify", payload)
    assert not log.observe("feeder_gui_notify", payload)
    assert not log.observe("feeder_system_state", "IDLE")
    assert log.observe("feeder_gui_notify", payload)

    rows = read_rows(path)
    assert len(rows) == 2
    assert rows[0]["Area"] == "FEEDER"
    assert rows[0]["Message"] == "Servo offline — InX timeout"


@pytest.mark.parametrize(
    ("key", "value", "area", "level"),
    [
        ("robot_system_status", "ERROR_GRIPPER", "ROBOT", "ERROR"),
        ("robot_system_status", "EMERGENCY_STOP", "ROBOT", "ERROR"),
        ("feeder_system_state", "ERROR", "FEEDER", "ERROR"),
        ("hw_status", "PLC DISCONNECTED", "FILL_HP", "ERROR"),
        ("scale_status", "ERROR: RS485 timeout", "SCALE", "ERROR"),
        ("scale_monitor_status", "NO_SIGNAL", "SCALE", "ERROR"),
        ("scale_cal_status", "CAL_ERROR", "SCALE", "ERROR"),
        ("scale_overload", True, "SCALE", "ERROR"),
        ("scale_zero_drift", True, "SCALE", "WARN"),
        ("vfd_status", "ERROR", "VFD", "ERROR"),
        ("camera_status", "CAMERA_SWITCH_FAILED", "CAMERA", "ERROR"),
        ("camera_status", "corr_err: discarding frame", "CAMERA", "ERROR"),
        ("camera_cam0_health", "FATAL_WATCHDOG", "CAMERA", "ERROR"),
        ("camera_cam1_health", "FATAL", "CAMERA", "ERROR"),
        ("camera_cam0_health", "RECONNECTING device=/dev/video0", "CAMERA", "WARN"),
        ("vision_roi_status", "ERROR: ROI unavailable", "CAMERA", "ERROR"),
        ("vision_roi_status", "ROI thieu slot: 7/8", "CAMERA", "ERROR"),
    ],
)
def test_critical_sources_are_captured(tmp_path, key, value, area, level):
    path = tmp_path / f"{key}.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)
    assert log.observe(key, value)
    row = read_rows(path)[0]
    assert row["Area"] == area
    assert row["Level"] == level


def test_fill_error_status_is_not_duplicated_external_to_system_stop_log(tmp_path):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)
    assert not log.observe("error_status", "ERROR: chamber pressure timeout")
    assert read_rows(path) == []


def test_normal_failed_cartridge_is_not_a_system_stop(tmp_path):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)
    assert not log.observe("scale_monitor_status", "LAST:FAIL total=12")
    assert not log.observe("scale_consecutive_fails", 3)
    assert read_rows(path) == []


def test_changing_camera_metrics_do_not_flood_one_reconnect_incident(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)

    assert log.observe(
        "camera_cam0_health",
        "RECONNECTING device=/dev/video0 fps=0.0 age=4.00s reconnects=1",
    )
    clock.advance(1)
    assert not log.observe(
        "camera_cam0_health",
        "RECONNECTING device=/dev/video0 fps=0.0 age=5.00s reconnects=2",
    )
    rows = read_rows(path)
    assert len(rows) == 1
    assert rows[0]["Message"] == "CAM0 RECONNECTING device=/dev/video0"


def test_persistent_dedup_suppresses_latched_message_after_restart(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    first = ExternalEventLog(path=str(path), now_fn=clock.now)
    assert first.observe("robot_error", "ERROR_DOBOT_OFFLINE")

    restarted = ExternalEventLog(path=str(path), now_fn=clock.now)
    assert not restarted.observe("robot_error", "ERROR_DOBOT_OFFLINE")
    assert len(read_rows(path)) == 1


def test_summary_and_merge_recalculate_counts_and_remove_exact_duplicate(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)
    assert log.observe("camera_status", "CAMERA TIMEOUT")
    clock.advance(1)
    assert log.observe("scale_zero_drift", True)
    external = log.summary_day("2026-08-05")

    duplicate = dict(external["items"][0])
    primary = {
        "date": "2026-08-05",
        "count": 1,
        "error": 1,
        "warn": 0,
        "items": [duplicate],
    }
    merged = merge_event_summaries(primary, external)
    assert merged["count"] == 2
    assert merged["error"] == 1
    assert merged["warn"] == 1
    assert [item["area"] for item in merged["items"]] == ["CAMERA", "SCALE"]


def test_csv_contract_is_external_and_has_expected_header(tmp_path):
    path = tmp_path / "external_system_events.csv"
    ExternalEventLog(path=str(path), now_fn=Clock().now)
    with path.open(newline="", encoding="utf-8") as stream:
        assert next(csv.reader(stream)) == EVENT_HEADER
    assert path.name != "system_stop_events.csv"
