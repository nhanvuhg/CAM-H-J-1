#!/usr/bin/env python3
"""Apply the AUTO/AI-only Stop Errors policy to production_log.py.

Check mode is read-only. ``--apply`` creates a timestamped backup and replaces
the source atomically. No ROS node or service is restarted by this script.
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


PATCH_ID = "CAM_HJ1_PRODUCTION_STOP_POLICY_V1"
MARKER = f"BEGIN {PATCH_ID}"

ANCHOR = '''        """Ghi lịch sử các lỗi buộc dừng/chạy lại hệ thống."""
        try:
'''

REPLACEMENT = f'''        """Ghi lịch sử các lỗi buộc dừng/chạy lại hệ thống."""
        # BEGIN {PATCH_ID}
        normalized_level = str(level or "ERROR").strip().upper()
        normalized_mode = " ".join(
            str(mode or "").strip().upper().replace("_", " ").replace("-", " ").split()
        )
        mode_head = normalized_mode.split(" ", 1)[0] if normalized_mode else ""
        if normalized_level != "ERROR" or mode_head not in ("AUTO", "AI", "1", "2"):
            self._warn(
                f"Stop event chỉ cảnh báo runtime, không lưu Production Output: "
                f"level={{normalized_level}} mode={{normalized_mode or 'UNKNOWN'}} "
                f"area={{area}} - {{message}}"
            )
            return False
        # END {PATCH_ID}
        try:
'''


class PatchError(RuntimeError):
    pass


def patch_source(source: str) -> Tuple[str, bool]:
    if MARKER in source:
        compile(source, "production_log.py", "exec")
        return source, False
    count = source.count(ANCHOR)
    if count != 1:
        raise PatchError(f"Expected exactly one stop-event anchor, found {count}")
    patched = source.replace(ANCHOR, REPLACEMENT, 1)
    compile(patched, "production_log.py", "exec")
    return patched, True


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
    target: Path, backup_dir: Path, apply: bool = False
) -> Dict[str, object]:
    if not target.is_file():
        raise PatchError(f"production_log.py not found: {target}")
    original = target.read_text(encoding="utf-8")
    patched, changed = patch_source(original)
    result: Dict[str, object] = {
        "mode": "apply" if apply else "check",
        "target": str(target),
        "patch_id": PATCH_ID,
        "source_changed": changed,
        "service_restarted": False,
        "backups": [],
    }
    if not apply or not changed:
        return result

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = _backup(target, backup_dir, stamp)
    _atomic_write(target, patched.encode("utf-8"), target.stat().st_mode)
    installed = target.read_text(encoding="utf-8")
    compile(installed, str(target), "exec")
    if MARKER not in installed:
        raise PatchError("production_log.py verification failed")
    result["backups"] = [str(backup)]
    result["target_sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True, type=Path)
    parser.add_argument("--backup-dir", required=True, type=Path)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    try:
        result = inspect_or_apply(args.target, args.backup_dir, args.apply)
    except (OSError, PatchError, SyntaxError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps({"ok": True, **result}, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
