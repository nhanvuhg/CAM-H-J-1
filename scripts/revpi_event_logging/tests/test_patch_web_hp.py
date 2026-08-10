from pathlib import Path

from patch_web_hp import (
    CONTEXT_MODE_NEW,
    CONTEXT_MODE_OLD,
    MARKERS,
    inspect_or_apply,
    patch_source,
)


def web_hp_fixture():
    return '''import os
try:
    from fill_hp import production_log as prodlog
except ImportError:
    import production_log as prodlog


BASE_DIR = os.path.dirname(__file__)

class FillHpWebNode:
    def __init__(self):
        self._status = {
            "scale_ink_capacity_ack": None,
        }

        self.pub_manual = None
        self.create_subscription(Float32, "/Fill_HP1/ink_capacity_ack", self._float_cb("scale_ink_capacity_ack"), 10)

    def _string_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = msg.data
                if key == "input_state":
                    self._sync_input_flags(msg.data)
        return cb

    def _sync_input_flags(self, value):
        pass

    def _int_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = int(msg.data)
        return cb

    def _bool_cb(self, key):
        def cb(msg):
            with self._lock:
                self._status[key] = bool(msg.data)
        return cb

    def snapshot(self):
        return self._status

def make_handler(node):
    class Handler:
        def do_GET(self):
            path = self.path
            if path == "/before":
                pass
            elif path == "/events/today":
                self._send(200, prodlog.system_events_today())
            elif path == "/events/date":
                self._send(200, prodlog.system_events_day(self._qget("date") or prodlog.today_str()))
    return Handler
'''


def test_patch_source_is_syntactically_valid_and_idempotent():
    patched, changed = patch_source(web_hp_fixture())
    assert changed
    compile(patched, "web_hp.py", "exec")
    for marker in MARKERS:
        assert marker in patched
    assert '"/vfd/status"' in patched
    assert '"/camera/status"' in patched
    assert '"/vision/roi_status", self._string_cb("vision_roi_status"), qos_latched' in patched
    assert "node.merged_system_events_day" in patched
    assert CONTEXT_MODE_NEW in patched

    second, changed_again = patch_source(patched)
    assert not changed_again
    assert second == patched


def test_existing_v1_patch_migrates_mode_context_once():
    patched, _ = patch_source(web_hp_fixture())
    old_v1 = patched.replace(CONTEXT_MODE_NEW, CONTEXT_MODE_OLD, 1)

    migrated, changed = patch_source(old_v1)
    assert changed
    assert CONTEXT_MODE_NEW in migrated
    assert CONTEXT_MODE_OLD not in migrated

    repeated, changed_again = patch_source(migrated)
    assert not changed_again
    assert repeated == migrated


def test_check_mode_is_read_only_then_apply_creates_backup(tmp_path):
    target = tmp_path / "web_hp.py"
    target.write_text(web_hp_fixture(), encoding="utf-8")
    original = target.read_bytes()
    module_source = Path(__file__).resolve().parents[1] / "external_event_log.py"
    backups = tmp_path / "backups"

    checked = inspect_or_apply(target, module_source, backups, apply=False)
    assert checked["source_changed"]
    assert target.read_bytes() == original
    assert not (tmp_path / "external_event_log.py").exists()

    applied = inspect_or_apply(target, module_source, backups, apply=True)
    assert applied["source_changed"]
    assert applied["module_changed"]
    assert applied["service_restarted"] is False
    assert len(applied["backups"]) == 1  # new module has no previous file
    assert Path(applied["backups"][0]).read_bytes() == original
    assert (tmp_path / "external_event_log.py").read_bytes() == module_source.read_bytes()

    repeated = inspect_or_apply(target, module_source, backups, apply=True)
    assert not repeated["source_changed"]
    assert not repeated["module_changed"]
    assert repeated["backups"] == []


def test_module_only_installs_import_before_web_patch(tmp_path):
    target = tmp_path / "web_hp.py"
    target.write_text(web_hp_fixture(), encoding="utf-8")
    original = target.read_text(encoding="utf-8")
    module_source = Path(__file__).resolve().parents[1] / "external_event_log.py"

    result = inspect_or_apply(
        target,
        module_source,
        tmp_path / "backups",
        apply=True,
        module_only=True,
    )
    assert result["module_only"] is True
    assert not result["source_changed"]
    assert target.read_text(encoding="utf-8") == original
    assert (tmp_path / "external_event_log.py").is_file()
