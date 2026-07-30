# CAM-H-J-1 ROS 2 workspace

Workspace điều khiển và giám sát chạy trên NVIDIA Jetson với ROS 2 Humble,
giao tiếp RevPi/ROS 2, hai camera CSI, Festo CPX, robot và GUI Qt/QML.

## Clone và build

```bash
git clone https://github.com/nhanvuhg/CAM-H-J-1.git ros2_ws
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Các dependency nguồn cần thiết đã được lưu trực tiếp trong repository để một
commit/tag luôn khôi phục được đúng bộ source, không phụ thuộc revision
submodule bên ngoài.

Tạo file tài khoản cục bộ trước khi chạy GUI:

```bash
cp src/unified_control_gui/fill_hp_users.example.json \
   src/unified_control_gui/fill_hp_users.json
chmod 600 src/unified_control_gui/fill_hp_users.json
```

Sửa username/password trong file vừa tạo. File thật được Git bỏ qua và không
được push lên repository public.

Chạy toàn hệ thống:

```bash
bash scripts/start_all.sh
```

Quy trình cập nhật và rollback được ghi tại
[docs/GIT_UPDATE_ROLLBACK.md](docs/GIT_UPDATE_ROLLBACK.md).
