#!/usr/bin/env python3
"""Idempotently add external error capture to RevPi A ``web_hp.py``.

The default mode is read-only.  Pass ``--apply`` to create timestamped backups,
install ``external_event_log.py`` beside the target, and atomically replace the
patched source.  This script never builds or restarts a service.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import stat
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Tuple


PATCH_ID = "CAM_HJ1_EXTERNAL_EVENT_LOG_V1"
MARKERS = (
    f"BEGIN {PATCH_ID}_IMPORT",
    f"BEGIN {PATCH_ID}_INIT",
    f"BEGIN {PATCH_ID}_CALLBACKS",
    f"BEGIN {PATCH_ID}_API",
)

IMPORT_ANCHOR = """    import production_log as prodlog


BASE_DIR"""
IMPORT_REPLACEMENT = f"""    import production_log as prodlog

# BEGIN {PATCH_ID}_IMPORT
try:
    from fill_hp import external_event_log as extevent
except ImportError:
    import external_event_log as extevent
# END {PATCH_ID}_IMPORT


BASE_DIR"""

STATUS_ANCHOR = """            "scale_ink_capacity_ack": None,
        }

        self.pub_manual"""
STATUS_REPLACEMENT = f"""            "scale_ink_capacity_ack": None,
            # BEGIN {PATCH_ID}_INIT
            "vfd_status": "",
            "camera_status": "",
            "camera_cam0_health": "",
            "camera_cam1_health": "",
            "vision_roi_status": "",
            # END {PATCH_ID}_INIT
        }}

        self._external_event_log = extevent.ExternalEventLog(
            logger=self.get_logger(),
            now_fn=lambda: prodlog.trusted_now_local(self.get_logger()),
        )

        self.pub_manual"""

SUBSCRIPTION_ANCHOR = """        self.create_subscription(Float32, "/Fill_HP1/ink_capacity_ack", self._float_cb("scale_ink_capacity_ack"), 10)

    def _string_cb(self, key):"""
SUBSCRIPTION_REPLACEMENT = f"""        self.create_subscription(Float32, "/Fill_HP1/ink_capacity_ack", self._float_cb("scale_ink_capacity_ack"), 10)
        self.create_subscription(String, "/vfd/status", self._string_cb("vfd_status"), 10)
        self.create_subscription(String, "/camera/status", self._string_cb("camera_status"), 10)
        self.create_subscription(String, "/camera/cam0/health", self._string_cb("camera_cam0_health"), 10)
        self.create_subscription(String, "/camera/cam1/health", self._string_cb("camera_cam1_health"), 10)
        self.create_subscription(String, "/vision/roi_status", self._string_cb("vision_roi_status"), qos_latched)

    # BEGIN {PATCH_ID}_CALLBACKS
    def _event_context(self):
        with self._lock:
            status = dict(self._status)
        return {{
            "mode": status.get("mode_status") or status.get("feeder_current_mode") or "",
            "fill_state": status.get("fill_status") or "",
            "dosing_state": status.get("dosing_status") or "",
            "cr_state": status.get("cr_status") or "",
        }}

    def _observe_external_event(self, key, value):
        try:
            self._external_event_log.observe(key, value, self._event_context())
        except Exception as exc:  # Logging must never stop a ROS callback.
            self.get_logger().warn(f"External event capture failed [{{key}}]: {{exc}}")

    def _string_cb(self, key):"""

STRING_CALLBACK_ANCHOR = """    def _string_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = msg.data
                if key == "input_state":
                    self._sync_input_flags(msg.data)
        return cb
"""
STRING_CALLBACK_REPLACEMENT = """    def _string_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = msg.data
                if key == "input_state":
                    self._sync_input_flags(msg.data)
            self._observe_external_event(key, msg.data)
        return cb
"""

INT_CALLBACK_ANCHOR = """    def _int_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = int(msg.data)
        return cb
"""
INT_CALLBACK_REPLACEMENT = """    def _int_cb(self, key):
        def cb(msg):
            value = int(msg.data)
            with self._lock:
                self._status[key] = value
            self._observe_external_event(key, value)
        return cb
"""

BOOL_CALLBACK_ANCHOR = """    def _bool_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = bool(msg.data)
        return cb
"""
BOOL_CALLBACK_REPLACEMENT = """    def _bool_cb(self, key):
        def cb(msg):
            value = bool(msg.data)
            with self._lock:
                self._status[key] = value
            self._observe_external_event(key, value)
        return cb
"""

SNAPSHOT_ANCHOR = """    def snapshot(self):
"""
SNAPSHOT_REPLACEMENT = f"""    def merged_system_events_day(self, date_str):
        primary = prodlog.system_events_day(date_str)
        try:
            external = self._external_event_log.summary_day(date_str)
            return extevent.merge_event_summaries(primary, external)
        except Exception as exc:
            self.get_logger().warn(f"External event merge failed: {{exc}}")
            return primary

    def merged_system_events_today(self):
        return self.merged_system_events_day(prodlog.today_str())
    # END {PATCH_ID}_CALLBACKS

    def snapshot(self):
"""

TODAY_API_ANCHOR = """            elif path == "/events/today":
                self._send(200, prodlog.system_events_today())
            elif path == "/events/date":
                self._send(200, prodlog.system_events_day(self._qget("date") or prodlog.today_str()))
"""
TODAY_API_REPLACEMENT = f"""            # BEGIN {PATCH_ID}_API
            elif path == "/events/today":
                self._send(200, node.merged_system_events_today())
            elif path == "/events/date":
                self._send(200, node.merged_system_events_day(
                    self._qget("date") or prodlog.today_str()))
            # END {PATCH_ID}_API
"""


class PatchError(RuntimeError):
    pass


def _replace_once(source: str, anchor: str, replacement: str, label: str) -> str:
    count = source.count(anchor)
    if count != 1:
        raise PatchError(f"Expected exactly one {label} anchor, found {count}")
    return source.replace(anchor, replacement, 1)


def patch_source(source: str) -> Tuple[str, bool]:
    """Return ``(patched_source, changed)`` without touching the filesystem."""
    marker_presence = [marker in source for marker in MARKERS]
    if all(marker_presence):
        compile(source, "web_hp.py", "exec")
        return source, False
    if any(marker_presence):
        present = [m for m, found in zip(MARKERS, marker_presence) if found]
        raise PatchError(f"Partial {PATCH_ID} patch detected: {present}")

    patched = source
    patched = _replace_once(patched, IMPORT_ANCHOR, IMPORT_REPLACEMENT, "import")
    patched = _replace_once(patched, STATUS_ANCHOR, STATUS_REPLACEMENT, "status/init")
    patched = _replace_once(
        patched, SUBSCRIPTION_ANCHOR, SUBSCRIPTION_REPLACEMENT, "subscriptions"
    )
    patched = _replace_once(
        patched, STRING_CALLBACK_ANCHOR, STRING_CALLBACK_REPLACEMENT, "string callback"
    )
    patched = _replace_once(
        patched, INT_CALLBACK_ANCHOR, INT_CALLBACK_REPLACEMENT, "int callback"
    )
    patched = _replace_once(
        patched, BOOL_CALLBACK_ANCHOR, BOOL_CALLBACK_REPLACEMENT, "bool callback"
    )
    patched = _replace_once(patched, SNAPSHOT_ANCHOR, SNAPSHOT_REPLACEMENT, "snapshot")
    patched = _replace_once(patched, TODAY_API_ANCHOR, TODAY_API_REPLACEMENT, "event API")
    compile(patched, "web_hp.py", "exec")
    return patched, True


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _backup(path: Path, backup_dir: Path, stamp: str) -> Path:
    backup_dir.mkdir(parents=True, exist_ok=True)
    candidate = backup_dir / f"{path.name}.{stamp}.bak"
    suffix = 1
    while candidate.exists():
        candidate = backup_dir / f"{path.name}.{stamp}.{suffix}.bak"
        suffix += 1
    shutil.copy2(path, candidate)
    return candidate


def _atomic_write(path: Path, data: bytes, mode: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(dir=str(path.parent), delete=False) as stream:
            temp_name = stream.name
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temp_name, stat.S_IMODE(mode))
        os.replace(temp_name, path)
        temp_name = None
    finally:
        if temp_name:
            try:
                os.unlink(temp_name)
            except OSError:
                pass


def inspect_or_apply(
    target: Path,
    module_source: Path,
    backup_dir: Path,
    apply: bool = False,
    module_only: bool = False,
) -> Dict[str, object]:
    if not target.is_file():
        raise PatchError(f"web_hp.py not found: {target}")
    if not module_source.is_file():
        raise PatchError(f"external_event_log.py not found: {module_source}")

    original = target.read_text(encoding="utf-8")
    if module_only:
        compile(original, str(target), "exec")
        patched, source_changed = original, False
    else:
        patched, source_changed = patch_source(original)
    module_bytes = module_source.read_bytes()
    compile(module_bytes, str(module_source), "exec")
    module_target = target.parent / "external_event_log.py"
    old_module_bytes = module_target.read_bytes() if module_target.exists() else b""
    module_changed = old_module_bytes != module_bytes

    result: Dict[str, object] = {
        "mode": "apply" if apply else "check",
        "module_only": module_only,
        "target": str(target),
        "patch_id": PATCH_ID,
        "source_changed": source_changed,
        "module_target": str(module_target),
        "module_changed": module_changed,
        "service_restarted": False,
        "backups": [],
    }
    if not apply:
        return result

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backups = []
    # All parsing/compilation checks above complete before the first write.
    if module_changed:
        if module_target.exists():
            backups.append(str(_backup(module_target, backup_dir, stamp)))
        _atomic_write(module_target, module_bytes, 0o644)
    if source_changed:
        backups.append(str(_backup(target, backup_dir, stamp)))
        target_mode = target.stat().st_mode
        _atomic_write(target, patched.encode("utf-8"), target_mode)

    # Verify exact bytes and syntax after replacement.  Do not import ROS here.
    installed = target.read_text(encoding="utf-8")
    compile(installed, str(target), "exec")
    if module_target.read_bytes() != module_bytes:
        raise PatchError("external_event_log.py verification failed")
    result["backups"] = backups
    result["target_sha256"] = _sha256(target.read_bytes())
    result["module_sha256"] = _sha256(module_target.read_bytes())
    return result


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, type=Path, help="Path to web_hp.py")
    parser.add_argument(
        "--module-source", required=True, type=Path, help="Tracked external_event_log.py"
    )
    parser.add_argument(
        "--backup-dir",
        type=Path,
        default=Path("/home/pi/cartridge_fill_logs/web_hp_backups"),
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Write changes. Without this flag the command is read-only.",
    )
    parser.add_argument(
        "--module-only",
        action="store_true",
        help="Install/verify only the companion module; leave web_hp.py unchanged.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        result = inspect_or_apply(
            target=args.target,
            module_source=args.module_source,
            backup_dir=args.backup_dir,
            apply=args.apply,
            module_only=args.module_only,
        )
    except (OSError, PatchError, SyntaxError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps({"ok": True, **result}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
