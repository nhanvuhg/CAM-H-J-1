#!/usr/bin/env python3
"""Persistent event capture used by the RevPi A web bridge.

This module deliberately writes to ``external_system_events.csv`` instead of
``system_stop_events.csv``.  The latter is owned by ``production_log.py`` and
records Fill HP stops; keeping the bridge events separate avoids changing or
corrupting that existing contract.

Only Python's standard library is used so the file can be copied next to
``web_hp.py`` and imported by either a ROS package install or a source launch.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import re
import threading
from datetime import datetime, timedelta, timezone
from typing import Any, Callable, Dict, Iterable, Mapping, Optional


EVENT_HEADER = [
    "Timestamp",
    "Date",
    "Machine",
    "Operator",
    "Level",
    "Area",
    "Mode",
    "Fill_State",
    "Dosing_State",
    "CR_State",
    "Message",
    "Action",
    "Source",
    "Fingerprint",
]

DEFAULT_LOG_DIR = os.environ.get(
    "FILL_HP_PROD_LOG_DIR", "/home/pi/cartridge_fill_logs"
)
DEFAULT_EVENT_CSV = os.environ.get(
    "FILL_HP_EXTERNAL_EVENT_CSV",
    os.path.join(DEFAULT_LOG_DIR, "external_system_events.csv"),
)
DEFAULT_MACHINE = os.environ.get("FILL_HP_MACHINE_NAME", "InkoBot360H")
LOCAL_TZ_OFFSET_HOURS = float(
    os.environ.get("FILL_HP_LOG_TZ_OFFSET_HOURS", "7")
)

_HEALTHY_EXACT = {
    "",
    "-",
    "0",
    "FALSE",
    "HEALTHY",
    "IDLE",
    "NONE",
    "NORMAL",
    "NO ERROR",
    "NO_ERROR",
    "ERROR_NONE",
    "OK",
    "ONLINE",
    "READY",
    "RUNNING",
    "STREAMING",
}
_CRITICAL_RE = re.compile(
    r"(?:^|[^A-Z0-9])(?:ERROR|FAULT|FATAL|FAILED?|FAILURE|TIMEOUT|OFFLINE|"
    r"DISCONNECT(?:ED)?|NO[ _]SIGNAL|CORR[ _]ERR|OVERLOAD|EMERGENCY)"
    r"(?:$|[^A-Z0-9])"
)
_ERROR_STATE_RE = re.compile(r"^(?:ERROR|FAULT)(?:$|[_:\s-])")

_RECORD_WRITTEN = "written"
_RECORD_DUPLICATE = "duplicate"
_RECORD_FAILED = "failed"
_RECORD_IGNORED = "ignored"

_ACTIONS = {
    "ROBOT": "Dừng chuyển động, kiểm tra robot và chỉ chạy lại sau khi lỗi đã được xử lý.",
    "FEEDER": "Kiểm tra cartridge feeder, servo và cảm biến trước khi tiếp tục.",
    "FILL_HP": "Kiểm tra phần cứng Fill HP và xác nhận trạng thái an toàn trước khi chạy lại.",
    "SCALE": "Kiểm tra cân/loadcell, kết nối RS485 và tải đặt trên cân.",
    "VFD": "Dừng băng tải và kiểm tra VFD, RS485 cùng tín hiệu fault trước khi chạy lại.",
    "CAMERA": "Kiểm tra camera, CSI/VI và topic hình ảnh trước khi tiếp tục.",
}


def _default_now() -> datetime:
    tz = timezone(timedelta(hours=LOCAL_TZ_OFFSET_HOURS))
    return datetime.now(timezone.utc).astimezone(tz)


def _clean(value: Any) -> str:
    return " ".join(str(value if value is not None else "").split())


def _upper(value: Any) -> str:
    return _clean(value).upper()


def _is_healthy(value: Any) -> bool:
    text = _upper(value)
    if text in _HEALTHY_EXACT:
        return True
    return (
        text.startswith("OK:")
        or text.startswith("READY:")
        or "NO ERROR" in text
        or "NO_ERROR" in text
        or "ERROR_NONE" in text
    )


def _has_critical_token(value: Any) -> bool:
    text = _upper(value).replace("_", " ")
    return not _is_healthy(value) and bool(_CRITICAL_RE.search(text))


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return _upper(value) in {"1", "ON", "TRUE", "YES"}


def _fingerprint(level: str, area: str, message: str) -> str:
    normalized = "|".join((_upper(level), _upper(area), _upper(message)))
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()[:24]


def _parse_timestamp(value: str, reference: datetime) -> Optional[datetime]:
    try:
        parsed = datetime.fromisoformat(str(value).strip())
    except (TypeError, ValueError):
        return None
    if parsed.tzinfo is None and reference.tzinfo is not None:
        parsed = parsed.replace(tzinfo=reference.tzinfo)
    return parsed


def _warn(logger: Any, message: str) -> None:
    if logger is None:
        return
    try:
        if hasattr(logger, "warn"):
            logger.warn(message)
        elif hasattr(logger, "warning"):
            logger.warning(message)
    except Exception:
        # Logging must never be able to stop a ROS callback.
        pass


class ExternalEventLog:
    """Append-only, transition-aware recorder for errors received by web_hp.

    Repeated samples of the same active fault are ignored.  A healthy sample
    clears that source, so a later transition back into the same fault is a new
    event.  The short persistent de-duplication window prevents a latched ROS
    message from being appended again immediately after web_hp restarts.
    """

    def __init__(
        self,
        path: str = DEFAULT_EVENT_CSV,
        machine: str = DEFAULT_MACHINE,
        logger: Any = None,
        now_fn: Optional[Callable[[], datetime]] = None,
        persistent_dedup_seconds: float = 10.0,
        incident_quiet_seconds: float = 30.0,
    ) -> None:
        self.path = os.path.abspath(path)
        self.machine = _clean(machine) or DEFAULT_MACHINE
        self.logger = logger
        self.now_fn = now_fn or _default_now
        self.persistent_dedup_seconds = max(0.0, float(persistent_dedup_seconds))
        self.incident_quiet_seconds = max(0.0, float(incident_quiet_seconds))
        self._lock = threading.Lock()
        # Missing key = first observation (check persistent de-duplication).
        # None = source recovered (a repeated fault is a real new transition).
        self._active: Dict[str, Optional[str]] = {}
        self._last_seen: Dict[str, datetime] = {}
        self._ensure_file()

    def _ensure_file(self) -> bool:
        try:
            parent = os.path.dirname(self.path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            if not os.path.exists(self.path) or os.path.getsize(self.path) == 0:
                with open(self.path, "w", newline="", encoding="utf-8") as stream:
                    csv.writer(stream).writerow(EVENT_HEADER)
                    stream.flush()
                    os.fsync(stream.fileno())
                return True
            with open(self.path, newline="", encoding="utf-8") as stream:
                header = next(csv.reader(stream), [])
            if header != EVENT_HEADER:
                _warn(
                    self.logger,
                    f"External event CSV header không hợp lệ, không ghi đè: {self.path}",
                )
                return False
            return True
        except OSError as exc:
            _warn(self.logger, f"Không tạo được external event log {self.path}: {exc}")
            return False

    def _recent_duplicate(self, fingerprint: str, now: datetime) -> bool:
        if self.persistent_dedup_seconds <= 0 or not os.path.exists(self.path):
            return False
        try:
            with open(self.path, newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
        except OSError:
            return False
        # Error files are expected to be small.  Limiting the scan also keeps a
        # damaged/very old installation from slowing a ROS callback.
        for row in reversed(rows[-256:]):
            if row.get("Fingerprint") != fingerprint:
                continue
            previous = _parse_timestamp(row.get("Timestamp", ""), now)
            if previous is None:
                continue
            age = (now - previous).total_seconds()
            return 0.0 <= age <= self.persistent_dedup_seconds
        return False

    def _record_outcome(
        self,
        *,
        level: str,
        area: str,
        message: str,
        source: str,
        context: Optional[Mapping[str, Any]] = None,
        action: str = "",
        check_persistent_duplicate: bool = True,
    ) -> str:
        message = _clean(message)
        if not message:
            return _RECORD_IGNORED
        level = _upper(level) or "ERROR"
        area = _upper(area) or "SYSTEM"
        source = _clean(source)
        context = dict(context or {})
        fingerprint = _fingerprint(level, area, message)
        try:
            now = self.now_fn()
            if not isinstance(now, datetime):
                raise TypeError("now_fn must return datetime")
        except Exception as exc:
            _warn(self.logger, f"Không lấy được thời gian external event: {exc}")
            return _RECORD_FAILED

        with self._lock:
            if not self._ensure_file():
                return _RECORD_FAILED
            if check_persistent_duplicate and self._recent_duplicate(fingerprint, now):
                return _RECORD_DUPLICATE
            row = [
                now.strftime("%Y-%m-%d %H:%M:%S"),
                now.strftime("%Y-%m-%d"),
                _clean(context.get("machine")) or self.machine,
                _clean(context.get("operator")),
                level,
                area,
                _clean(context.get("mode")),
                _clean(context.get("fill_state")),
                _clean(context.get("dosing_state")),
                _clean(context.get("cr_state")),
                message,
                _clean(action) or _ACTIONS.get(area, _ACTIONS["FILL_HP"]),
                source,
                fingerprint,
            ]
            try:
                with open(self.path, "a", newline="", encoding="utf-8") as stream:
                    csv.writer(stream).writerow(row)
                    stream.flush()
                    os.fsync(stream.fileno())
            except OSError as exc:
                _warn(self.logger, f"Ghi external_system_events.csv lỗi: {exc}")
                return _RECORD_FAILED
        return _RECORD_WRITTEN

    def record(
        self,
        *,
        level: str,
        area: str,
        message: str,
        source: str,
        context: Optional[Mapping[str, Any]] = None,
        action: str = "",
        check_persistent_duplicate: bool = True,
    ) -> bool:
        """Persist one event; return True only when a CSV row was appended."""
        return self._record_outcome(
            level=level,
            area=area,
            message=message,
            source=source,
            context=context,
            action=action,
            check_persistent_duplicate=check_persistent_duplicate,
        ) == _RECORD_WRITTEN

    def _classify(self, key: str, value: Any) -> Optional[Dict[str, str]]:
        key = str(key or "").strip()
        text = _clean(value)

        if key == "robot_error":
            if _is_healthy(text):
                return None
            return {"level": "ERROR", "area": "ROBOT", "message": text}

        if key == "robot_system_status":
            if _is_healthy(text) or not (
                _ERROR_STATE_RE.search(_upper(text)) or _has_critical_token(text)
            ):
                return None
            return {"level": "ERROR", "area": "ROBOT", "message": text}

        if key == "feeder_gui_notify":
            try:
                payload = json.loads(text)
            except (TypeError, ValueError, json.JSONDecodeError):
                return None
            if not isinstance(payload, dict) or _upper(payload.get("level")) != "ERROR":
                return None
            title = _clean(payload.get("title"))
            detail = _clean(payload.get("detail") or payload.get("message"))
            message = " — ".join(part for part in (title, detail) if part)
            return {
                "level": "ERROR",
                "area": "FEEDER",
                "message": message or "Cartridge feeder reported an error",
            }

        if key == "feeder_system_state":
            if not (_ERROR_STATE_RE.search(_upper(text)) or _has_critical_token(text)):
                return None
            return {"level": "ERROR", "area": "FEEDER", "message": text}

        # ``error_status`` belongs to production_log.py and is already written
        # to system_stop_events.csv. Capturing it here would duplicate one Fill
        # HP incident in the merged API, so only independent hw_status faults
        # are recorded by this bridge.
        if key == "hw_status":
            if _is_healthy(text):
                return None
            if _has_critical_token(text):
                return {"level": "ERROR", "area": "FILL_HP", "message": text}
            return None

        if key == "scale_monitor_status":
            # LAST:FAIL is a normal per-cartridge NG decision which the robot
            # routes to PLACE_TO_FAIL. It is production data, not a stopped
            # system. Current monitor faults (for example OVERLOAD) still pass.
            if _upper(text).startswith("LAST:"):
                return None
            if _is_healthy(text) or not _has_critical_token(text):
                return None
            return {"level": "ERROR", "area": "SCALE", "message": text}

        if key in {"scale_status", "scale_cal_status"}:
            if _is_healthy(text) or not _has_critical_token(text):
                return None
            return {"level": "ERROR", "area": "SCALE", "message": text}

        if key == "scale_overload":
            if not _as_bool(value):
                return None
            return {"level": "ERROR", "area": "SCALE", "message": "Loadcell overload"}

        if key == "scale_zero_drift":
            if not _as_bool(value):
                return None
            return {"level": "WARN", "area": "SCALE", "message": "Loadcell zero drift warning"}

        if key == "vfd_status":
            if _is_healthy(text) or not _has_critical_token(text):
                return None
            return {"level": "ERROR", "area": "VFD", "message": text}

        if key == "vision_roi_status":
            # Contract from vision_decision_node: empty means OK and every
            # non-empty payload is the aggregated ROI validation error.  The
            # message is often Vietnamese and therefore need not contain an
            # English ERROR/FAULT token.
            if _is_healthy(text):
                return None
            return {"level": "ERROR", "area": "CAMERA", "message": text}

        if key in {"camera_status", "camera_cam0_health", "camera_cam1_health"}:
            is_health = key in {"camera_cam0_health", "camera_cam1_health"}
            label = "CAM0" if key == "camera_cam0_health" else "CAM1"
            upper = _upper(text)
            # Health payloads contain changing fps/age/counters every second.
            # Keep the state/reason plus device only, otherwise one reconnect
            # incident would create a new fingerprint on every health tick.
            stable_message = text
            if is_health:
                state_and_reason = text.split(" device=", 1)[0]
                device_match = re.search(r"(?:^|\s)device=([^\s]+)", text)
                stable_message = f"{label} {state_and_reason}"
                if device_match:
                    stable_message += f" device={device_match.group(1)}"
            if upper.startswith("RECONNECT"):
                return {
                    "level": "WARN",
                    "area": "CAMERA",
                    "message": stable_message,
                }
            if _is_healthy(text) or not _has_critical_token(text):
                return None
            return {
                "level": "ERROR",
                "area": "CAMERA",
                "message": stable_message,
            }

        return None

    def observe(
        self,
        key: str,
        value: Any,
        context: Optional[Mapping[str, Any]] = None,
    ) -> bool:
        """Observe one status sample and append only on entry to a fault state."""
        event = self._classify(key, value)
        source_key = str(key or "")
        if event is None:
            # Preserve the key with a recovered marker so recurrence is not
            # suppressed by the cross-restart de-duplication window.
            if source_key in self._active:
                self._active[source_key] = None
            # robot/error and gui_notify are event streams and do not reliably
            # emit an empty value. Their corresponding healthy state transition
            # is the explicit incident boundary.
            if source_key == "robot_system_status":
                self._active["robot_error"] = None
            elif source_key == "feeder_system_state":
                self._active["feeder_gui_notify"] = None
            return False

        fingerprint = _fingerprint(
            event["level"], event["area"], event["message"]
        )
        try:
            observed_at = self.now_fn()
        except Exception:
            observed_at = _default_now()
        if self._active.get(source_key) == fingerprint:
            last_seen = self._last_seen.get(source_key)
            if last_seen is None:
                self._last_seen[source_key] = observed_at
                return False
            # A continuously repeated topic remains one incident. If the event
            # stream was quiet for long enough, the next identical message is a
            # new occurrence even if no explicit healthy sample was published.
            quiet_for = (observed_at - last_seen).total_seconds()
            if quiet_for <= self.incident_quiet_seconds:
                self._last_seen[source_key] = observed_at
                return False
        first_observation = source_key not in self._active
        outcome = self._record_outcome(
            level=event["level"],
            area=event["area"],
            message=event["message"],
            source=source_key,
            context=context,
            check_persistent_duplicate=first_observation,
        )
        # A persistent duplicate means the incident was already durably
        # stored before this process started, so latch it. On I/O failure keep
        # the old state and retry when the next sample arrives.
        if outcome in {_RECORD_WRITTEN, _RECORD_DUPLICATE}:
            self._active[source_key] = fingerprint
            self._last_seen[source_key] = observed_at
        return outcome == _RECORD_WRITTEN

    def summary_day(self, date_str: str) -> Dict[str, Any]:
        items = []
        try:
            with self._lock:
                if not os.path.exists(self.path):
                    rows = []
                else:
                    with open(self.path, newline="", encoding="utf-8") as stream:
                        rows = list(csv.DictReader(stream))
        except OSError as exc:
            _warn(self.logger, f"Đọc external_system_events.csv lỗi: {exc}")
            rows = []

        for row in rows:
            if row.get("Date") != date_str:
                continue
            items.append(
                {
                    "time": row.get("Timestamp", ""),
                    "machine": row.get("Machine", ""),
                    "operator": row.get("Operator", ""),
                    "level": _upper(row.get("Level")) or "ERROR",
                    "area": row.get("Area", ""),
                    "mode": row.get("Mode", ""),
                    "fill_state": row.get("Fill_State", ""),
                    "dosing_state": row.get("Dosing_State", ""),
                    "cr_state": row.get("CR_State", ""),
                    "message": row.get("Message", ""),
                    "action": row.get("Action", ""),
                    "source": row.get("Source", ""),
                    "fingerprint": row.get("Fingerprint", ""),
                }
            )
        return _event_summary(str(date_str), items)

    def summary_today(self) -> Dict[str, Any]:
        return self.summary_day(self.now_fn().strftime("%Y-%m-%d"))


def _event_summary(date_str: str, items: Iterable[Mapping[str, Any]]) -> Dict[str, Any]:
    normalized = [dict(item) for item in items]
    normalized.sort(key=lambda item: str(item.get("time") or ""))
    error = sum(1 for item in normalized if _upper(item.get("level")) == "ERROR")
    warn = sum(1 for item in normalized if _upper(item.get("level")) == "WARN")
    return {
        "date": str(date_str),
        "count": len(normalized),
        "error": error,
        "warn": warn,
        "items": normalized,
    }


def merge_event_summaries(
    primary: Optional[Mapping[str, Any]],
    external: Optional[Mapping[str, Any]],
) -> Dict[str, Any]:
    """Merge API summaries and recalculate totals without duplicate rows."""
    primary = dict(primary or {})
    external = dict(external or {})
    date_str = str(primary.get("date") or external.get("date") or "")
    merged = []
    seen = set()
    for summary in (primary, external):
        for original in summary.get("items", []) or []:
            item = dict(original)
            identity = (
                _clean(item.get("time")),
                _upper(item.get("level")),
                _upper(item.get("area")),
                _upper(item.get("message")),
            )
            if identity in seen:
                continue
            seen.add(identity)
            merged.append(item)
    return _event_summary(date_str, merged)


__all__ = [
    "DEFAULT_EVENT_CSV",
    "EVENT_HEADER",
    "ExternalEventLog",
    "merge_event_summaries",
]
