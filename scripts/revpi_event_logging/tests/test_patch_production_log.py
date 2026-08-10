from pathlib import Path

from patch_production_log import MARKER, inspect_or_apply, patch_source


def production_log_fixture():
    return '''class ProductionLog:
    def _warn(self, message):
        pass

    def log_system_stop_event(
        self,
        level,
        area,
        message,
        operator="",
        mode="",
        fill_state="",
        dosing_state="",
        cr_state="",
        action="stop",
    ):
        """Ghi lịch sử các lỗi buộc dừng/chạy lại hệ thống."""
        try:
            return True
        except Exception:
            return False
'''


def _load_class(source):
    namespace = {}
    exec(compile(source, "production_log.py", "exec"), namespace)
    return namespace["ProductionLog"]


def test_policy_allows_only_error_in_auto_or_ai():
    patched, changed = patch_source(production_log_fixture())
    assert changed
    assert MARKER in patched
    logger = _load_class(patched)()

    assert logger.log_system_stop_event("ERROR", "ROBOT", "timeout", mode="AUTO MODE")
    assert logger.log_system_stop_event("error", "ROBOT", "timeout", mode="ai")
    assert not logger.log_system_stop_event("ERROR", "ROBOT", "offline", mode="MANUAL MODE")
    assert not logger.log_system_stop_event("WARN", "CAMERA", "reconnect", mode="AUTO MODE")


def test_patch_is_idempotent():
    patched, _ = patch_source(production_log_fixture())
    repeated, changed = patch_source(patched)
    assert not changed
    assert repeated == patched


def test_check_is_read_only_and_apply_creates_backup(tmp_path):
    target = tmp_path / "production_log.py"
    target.write_text(production_log_fixture(), encoding="utf-8")
    original = target.read_bytes()
    backups = tmp_path / "backups"

    checked = inspect_or_apply(target, backups, apply=False)
    assert checked["source_changed"]
    assert target.read_bytes() == original

    applied = inspect_or_apply(target, backups, apply=True)
    assert applied["source_changed"]
    assert len(applied["backups"]) == 1
    assert Path(applied["backups"][0]).read_bytes() == original
    assert MARKER in target.read_text(encoding="utf-8")
