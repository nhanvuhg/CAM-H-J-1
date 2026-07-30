# Kế Hoạch Phát Triển Hệ Thống Cấp Khay (Future Plan)
> Khi muốn thực hiện: kêu AI đọc file này và implement.

---

## Plan 1: Bỏ Hoàn Toàn Sensor Simulation ✅ DONE (branch feature/manual-real-sensors)

**Đã thực hiện:**
- ✅ Xóa `_sim_sensors` dict + `_cb_sim` callback + subscriber `/providesystem/sim_sensor`
- ✅ `sensor()` và `_snap()` luôn đọc IO module (bỏ branch theo mode)
- ✅ `sensor_real()` giữ làm alias backward-compat
- ✅ Xóa `_sim_sensors.clear()` calls trong `_cb_mode` và `_cb_robot_mode`
- ✅ QML: rename "SENSOR SIMULATION" → "SENSOR SIGNAL DISPLAY", bỏ controls (All ON/OFF/Clear, Quick Preset), buttons thành LED indicator read-only
- ✅ C++ controller: xóa `simSensor/simAll/simClear` methods + `sim_sensor_pub_`
- ✅ HTML GUI (`cartridge_gui.py`): xóa endpoint `/api/sim_sensor` + JS functions (`tog/sAll/sClear/simPreset`), grid sensor read-only
- ✅ Tests: convert `node._sim_sensors = {...}` → `_set_sensors(node, ...)` helper inject vào `_io_sensor_cache`

**Behavior sau plan:** Manual mode đọc sensor thật từ IO module (giống auto). State 2/4 vẫn trigger được bằng button (pub `/robot/done_tray_input` / `/robot/done_tray_output`). State 1/3 không auto-trigger trong manual.
