from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def read(relative_path: str) -> str:
    return (WORKSPACE_ROOT / relative_path).read_text(encoding="utf-8")


def test_desktop_launcher_marks_gui_as_managed():
    launcher = read("scripts/start_all.sh")

    assert 'UNIFIED_GUI_MANAGED_RESTART=1 "$QML_BIN"' in launcher
    assert ': > "$LOG_QML"' in launcher
    assert '"$QML_BIN" >> "$LOG_QML" 2>&1 &' in launcher
    assert 'GUI_RESTART_FLAG="/tmp/unified_gui_restart_requested"' in launcher
    assert 'if wait "$PID_QML_GUI"' in launcher
    assert 'if [ "$GUI_EXIT" -eq 42 ] || [ -f "$GUI_RESTART_FLAG" ]' in launcher


def test_desktop_launcher_waits_for_qml_before_starting_cuda_camera():
    launcher = read("scripts/start_all.sh")

    gui_ready_wait = 'grep -q "GUI_READY"'
    camera_start = 'camera_stack_supervisor >> "$LOG_CAMERA" 2>&1 &'
    assert gui_ready_wait in launcher
    assert launcher.index(gui_ready_wait) < launcher.index(camera_start)
    assert 'if ! start_qml_gui; then' in launcher


def test_legacy_launcher_survives_deliberate_gui_exit_with_set_e():
    launcher = read("src/unified_control_gui/scripts/start_unified_system.sh")

    assert "set -euo pipefail" in launcher
    assert 'if wait "$PID_GUI"' in launcher
    assert 'if [ "$GUI_EXIT" -eq 42 ] || [ -f "$GUI_RESTART_FLAG" ]' in launcher


def test_node_restart_delegates_camera_to_the_active_supervisor():
    restart_script = read(
        "src/unified_control_gui/scripts/restart_system_nodes.sh"
    )

    assert 'CAMERA_SUPERVISOR_PIDFILE="/tmp/camera_stack_supervisor.pid"' in restart_script
    assert "if ! flock -n 9" in restart_script
    assert "if ! $camera_is_managed; then\n" in restart_script
    assert (
        'ros2 launch csi_camera dual_camera_system.launch.py > '
        '"$LOG_DIR/dual_camera_system.log" 2>&1 &' in restart_script
    )
    assert 'kill_pattern "csi_camera_node"' not in restart_script


def test_launcher_publishes_and_cleans_camera_supervisor_pid():
    launcher = read("scripts/start_all.sh")

    assert 'echo "$PID_CAMERA" > "$CAMERA_SUPERVISOR_PIDFILE"' in launcher
    assert 'rm -f "$PIDFILE" "$CAMERA_SUPERVISOR_PIDFILE"' in launcher
