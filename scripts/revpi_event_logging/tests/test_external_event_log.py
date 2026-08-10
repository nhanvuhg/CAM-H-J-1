import csv
from datetime import datetime, timedelta, timezone

import pytest

from external_event_log import EVENT_HEADER, ExternalEventLog, merge_event_summaries

AUTO_CONTEXT = {"mode": "auto"}
AI_CONTEXT = {"mode": "AI MODE"}


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

    assert log.observe("robot_error", "ERROR_PICK_TIMEOUT", AUTO_CONTEXT)
    for _ in range(100):
        clock.advance(0.1)
        assert not log.observe("robot_error", "ERROR_PICK_TIMEOUT", AUTO_CONTEXT)
    assert len(read_rows(path)) == 1

    # /robot/error is an event stream and need not emit an empty message. A
    # healthy /robot/system_status explicitly closes the incident.
    assert not log.observe("robot_system_status", "STANDBY", AUTO_CONTEXT)
    assert log.observe("robot_error", "ERROR_PICK_TIMEOUT", AUTO_CONTEXT)
    assert len(read_rows(path)) == 2


def test_quiet_gap_distinguishes_recurrence_without_healthy_sample(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(
        path=str(path), now_fn=clock.now, incident_quiet_seconds=30
    )

    assert log.observe("robot_error", "ERROR_NO_TRAY", AUTO_CONTEXT)
    clock.advance(2)
    assert not log.observe("robot_error", "ERROR_NO_TRAY", AUTO_CONTEXT)
    clock.advance(31)
    assert log.observe("robot_error", "ERROR_NO_TRAY", AUTO_CONTEXT)
    assert len(read_rows(path)) == 2


def test_failed_write_does_not_latch_and_lose_the_incident(tmp_path, monkeypatch):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)
    real_ensure = log._ensure_file

    monkeypatch.setattr(log, "_ensure_file", lambda: False)
    assert not log.observe("robot_error", "DOBOT_CONTINUE_FAILED", AUTO_CONTEXT)

    monkeypatch.setattr(log, "_ensure_file", real_ensure)
    assert log.observe("robot_error", "DOBOT_CONTINUE_FAILED", AUTO_CONTEXT)
    assert len(read_rows(path)) == 1


def test_feeder_error_notification_is_transition_aware_and_state_recovers_it(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)
    payload = '{"level":"error","title":"Servo offline","detail":"InX timeout"}'

    assert log.observe("feeder_gui_notify", payload, AUTO_CONTEXT)
    assert not log.observe("feeder_gui_notify", payload, AUTO_CONTEXT)
    assert not log.observe("feeder_system_state", "IDLE", AUTO_CONTEXT)
    assert log.observe("feeder_gui_notify", payload, AUTO_CONTEXT)

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
        ("vfd_status", "ERROR", "VFD", "ERROR"),
        ("camera_status", "CAMERA_SWITCH_FAILED", "CAMERA", "ERROR"),
        ("camera_status", "corr_err: discarding frame", "CAMERA", "ERROR"),
        ("camera_cam0_health", "FATAL_WATCHDOG", "CAMERA", "ERROR"),
        ("camera_cam1_health", "FATAL", "CAMERA", "ERROR"),
        ("vision_roi_status", "ERROR: ROI unavailable", "CAMERA", "ERROR"),
        ("vision_roi_status", "ROI thieu slot: 7/8", "CAMERA", "ERROR"),
    ],
)
def test_critical_sources_are_captured(tmp_path, key, value, area, level):
    path = tmp_path / f"{key}.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)
    assert log.observe(key, value, AI_CONTEXT)
    row = read_rows(path)[0]
    assert row["Area"] == area
    assert row["Level"] == level


@pytest.mark.parametrize("mode", ["manual", "MANUAL MODE", "jog", "", "unknown"])
def test_manual_or_unknown_mode_errors_are_not_persisted(tmp_path, mode):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)

    assert not log.observe(
        "robot_error", "ERROR_DOBOT_OFFLINE", {"mode": mode}
    )
    assert not log.observe(
        "camera_status", "CAMERA TIMEOUT", {"mode": mode}
    )
    assert read_rows(path) == []


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("scale_zero_drift", True),
        ("camera_cam0_health", "RECONNECTING device=/dev/video0"),
    ],
)
def test_warning_events_are_not_persisted_even_in_auto(tmp_path, key, value):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)

    assert not log.observe(key, value, AUTO_CONTEXT)
    assert read_rows(path) == []


def test_manual_fault_can_be_recorded_if_it_persists_after_switch_to_auto(tmp_path):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)

    assert not log.observe(
        "robot_error", "ERROR_DOBOT_OFFLINE", {"mode": "manual"}
    )
    assert log.observe("robot_error", "ERROR_DOBOT_OFFLINE", AUTO_CONTEXT)
    assert len(read_rows(path)) == 1


def test_fill_error_status_is_not_duplicated_external_to_system_stop_log(tmp_path):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)
    assert not log.observe(
        "error_status", "ERROR: chamber pressure timeout", AUTO_CONTEXT
    )
    assert read_rows(path) == []


def test_normal_failed_cartridge_is_not_a_system_stop(tmp_path):
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=Clock().now)
    assert not log.observe(
        "scale_monitor_status", "LAST:FAIL total=12", AUTO_CONTEXT
    )
    assert not log.observe("scale_consecutive_fails", 3, AUTO_CONTEXT)
    assert read_rows(path) == []


def test_changing_camera_metrics_do_not_flood_one_reconnect_incident(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)

    assert not log.observe(
        "camera_cam0_health",
        "RECONNECTING device=/dev/video0 fps=0.0 age=4.00s reconnects=1",
        AUTO_CONTEXT,
    )
    clock.advance(1)
    assert not log.observe(
        "camera_cam0_health",
        "RECONNECTING device=/dev/video0 fps=0.0 age=5.00s reconnects=2",
        AUTO_CONTEXT,
    )
    rows = read_rows(path)
    assert rows == []


def test_persistent_dedup_suppresses_latched_message_after_restart(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    first = ExternalEventLog(path=str(path), now_fn=clock.now)
    assert first.observe("robot_error", "ERROR_DOBOT_OFFLINE", AUTO_CONTEXT)

    restarted = ExternalEventLog(path=str(path), now_fn=clock.now)
    assert not restarted.observe(
        "robot_error", "ERROR_DOBOT_OFFLINE", AUTO_CONTEXT
    )
    assert len(read_rows(path)) == 1


def test_summary_and_merge_recalculate_counts_and_remove_exact_duplicate(tmp_path):
    clock = Clock()
    path = tmp_path / "external_system_events.csv"
    log = ExternalEventLog(path=str(path), now_fn=clock.now)
    assert log.observe("camera_status", "CAMERA TIMEOUT", AUTO_CONTEXT)
    clock.advance(1)
    assert log.observe("robot_error", "ERROR_MOTION_TIMEOUT", AI_CONTEXT)
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
    assert merged["error"] == 2
    assert merged["warn"] == 0
    assert [item["area"] for item in merged["items"]] == ["CAMERA", "ROBOT"]


def test_merge_hides_legacy_manual_and_warning_rows():
    primary = {
        "date": "2026-08-05",
        "items": [
            {
                "time": "2026-08-05 12:00:00",
                "level": "ERROR",
                "area": "ROBOT",
                "mode": "MANUAL MODE",
                "message": "Disconnected during maintenance",
            },
            {
                "time": "2026-08-05 12:01:00",
                "level": "WARN",
                "area": "CAMERA",
                "mode": "AUTO MODE",
                "message": "Reconnect attempt",
            },
            {
                "time": "2026-08-05 12:02:00",
                "level": "ERROR",
                "area": "ROBOT",
                "mode": "AI MODE",
                "message": "Motion timeout",
            },
        ],
    }

    merged = merge_event_summaries(primary, None)
    assert merged["count"] == 1
    assert merged["error"] == 1
    assert merged["warn"] == 0
    assert merged["items"][0]["message"] == "Motion timeout"


def test_csv_contract_is_external_and_has_expected_header(tmp_path):
    path = tmp_path / "external_system_events.csv"
    ExternalEventLog(path=str(path), now_fn=Clock().now)
    with path.open(newline="", encoding="utf-8") as stream:
        assert next(csv.reader(stream)) == EVENT_HEADER
    assert path.name != "system_stop_events.csv"
