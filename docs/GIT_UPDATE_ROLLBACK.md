# Cập nhật và rollback workspace

Repository dùng nhánh `main` cho bản đang triển khai và tag để đánh dấu từng
mốc đã kiểm tra trên Jetson. File tài khoản thật, build, install và log chỉ nằm
trên máy, không thuộc Git.

## Lưu một mốc ổn định

Chỉ tạo mốc sau khi đã build và kiểm tra thiết bị:

```bash
cd ~/ros2_ws
git status
git add -A
git commit -m "Mô tả thay đổi đã kiểm tra"
git tag -a jetson-stable-YYYYMMDD-HHMM -m "Jetson stable YYYY-MM-DD HH:MM"
git push origin main
git push origin jetson-stable-YYYYMMDD-HHMM
```

Kiểm tra kỹ `git status` trước khi commit. Không dùng `git add -f` cho file tài
khoản hoặc khóa bí mật.

## Cập nhật Jetson

```bash
cd ~/ros2_ws
git status
git fetch origin --tags
git pull --ff-only origin main
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`--ff-only` dừng lại nếu máy có lịch sử khác với GitHub, tránh tự động tạo
merge khó kiểm soát.

## Rollback an toàn để kiểm tra

Xem các mốc:

```bash
git tag --sort=-creatordate
git log --oneline --decorate -20
```

Tạo một nhánh rollback từ tag, không xóa lịch sử hiện tại:

```bash
git switch -c rollback/kiem-tra jetson-stable-YYYYMMDD-HHMM
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Quay lại bản mới:

```bash
git switch main
```

Nếu một commit đã push cần được hủy trên `main`, dùng `git revert <commit>` rồi
push commit revert. Cách này giữ toàn bộ lịch sử và phù hợp cho máy đang triển
khai.

Không dùng `git clean -fdx`: lệnh đó có thể xóa file tài khoản cục bộ cùng toàn
bộ build/install.
