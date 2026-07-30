#!/usr/bin/env bash
# Đổi IP RevPi A cho toàn bộ cấu hình runtime của hệ thống.
#
# Cách dùng thông thường:
#   1. Sửa duy nhất REVPI_NEW_IP bên dưới.
#   2. Chạy file này từ Desktop.
#   3. Chạy "Start All ROS2".
#
# Có thể kiểm tra trước mà không sửa file:
#   bash update_revpi_ip.sh --dry-run
set -euo pipefail

# ╔══════════════════════════════════════════════════════════════╗
# ║  CHỈ CẦN SỬA IP MỚI CỦA REVPI Ở DÒNG NÀY                  ║
# ╚══════════════════════════════════════════════════════════════╝
REVPI_NEW_IP="172.16.11.31"

WS="${ROS2_WS:-$HOME/ros2_ws}"
ENV_FILE="$WS/ros2_env.sh"
XML_FILE="$WS/fastdds_peers.xml"
DESKTOP_START_ALL="$HOME/start_all.sh"
DEPLOY_SCRIPT="$HOME/deploy_revpi.sh"
REVPI_USER="${REVPI_USER:-pi}"
DRY_RUN=0
LOCAL_ONLY=0

usage() {
    cat <<'EOF'
Usage: update_revpi_ip.sh [NEW_IP] [--dry-run] [--local-only]

Không truyền NEW_IP: dùng biến REVPI_NEW_IP ở đầu file.
--dry-run:        chỉ hiển thị file sẽ thay đổi.
--local-only:     cập nhật Raspberry Pi 5, không copy config sang RevPi.
EOF
}

for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --local-only) LOCAL_ONLY=1 ;;
        -h|--help) usage; exit 0 ;;
        *)
            if [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                REVPI_NEW_IP="$arg"
            else
                echo "❌ Tham số không hợp lệ: $arg"
                usage
                exit 2
            fi
            ;;
    esac
done

valid_ipv4() {
    local ip="$1" octet
    [[ "$ip" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
    IFS='.' read -r -a octets <<< "$ip"
    for octet in "${octets[@]}"; do
        (( 10#$octet >= 0 && 10#$octet <= 255 )) || return 1
    done
    [[ "$ip" != "0.0.0.0" && "$ip" != "255.255.255.255" ]]
}

if ! valid_ipv4 "$REVPI_NEW_IP"; then
    echo "❌ REVPI_NEW_IP không phải IPv4 hợp lệ: $REVPI_NEW_IP"
    exit 2
fi

for required in "$ENV_FILE" "$XML_FILE"; do
    if [ ! -f "$required" ]; then
        echo "❌ Không tìm thấy file bắt buộc: $required"
        exit 1
    fi
done

OLD_ENV_IP="$(sed -nE 's/^export REVPI_A_HOST=["'\'']?([^"'\'']+)["'\'']?$/\1/p' "$ENV_FILE" | head -n1)"
if ! valid_ipv4 "${OLD_ENV_IP:-}"; then
    echo "❌ Không đọc được REVPI_A_HOST hiện tại từ $ENV_FILE"
    exit 1
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_DIR="$WS/backups/revpi_ip_$TIMESTAMP"

declare -a TARGETS=(
    "$ENV_FILE"
    "$XML_FILE"
    "$WS/scripts/start_all.sh"
    "$WS/restart_revpi_rs485.sh"
    "$WS/src/system_feed_cartridge/scripts/cartridge_gui_web.py"
    "$DEPLOY_SCRIPT"
    "$DESKTOP_START_ALL"
)

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Cập nhật IP RevPi A"
echo "  Env hiện tại : $OLD_ENV_IP"
echo "  IP mới       : $REVPI_NEW_IP"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "🔎 Quét các file runtime dùng bởi Start All:"
for file in "${TARGETS[@]}"; do
    [ -f "$file" ] && echo "   • $file"
done

if [ "$DRY_RUN" -eq 1 ]; then
    echo ""
    echo "🧪 Dry-run: chưa sửa file và chưa kết nối RevPi."
    exit 0
fi

mkdir -p "$BACKUP_DIR"

backup_file() {
    local file="$1"
    [ -f "$file" ] || return 0
    cp -a "$file" "$BACKUP_DIR/$(echo "$file" | sed 's#^/##; s#/#__#g')"
}

replace_with_sed() {
    local file="$1" expression="$2" tmp
    [ -f "$file" ] || return 0
    backup_file "$file"
    tmp="$(mktemp "${file}.XXXXXX")"
    sed -E "$expression" "$file" > "$tmp"
    chmod --reference="$file" "$tmp"
    mv "$tmp" "$file"
}

# 1) Single source of truth cho shell, GUI và SSH.
replace_with_sed "$ENV_FILE" \
    "s#^export REVPI_A_HOST=.*#export REVPI_A_HOST=\"$REVPI_NEW_IP\"#"

# 2) FastDDS không hỗ trợ env-var trong InitialPeersList. Lấy tất cả địa chỉ
# RevPi cũ (nhưng không đụng IP Raspberry Pi 5) rồi cập nhật cả comment + locator.
PI5_IP="$(sed -nE 's/^[[:space:]]*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)[[:space:]]*=[[:space:]]*Raspberry Pi 5.*/\1/p' "$XML_FILE" | head -n1)"
mapfile -t XML_REVPI_IPS < <(
    {
        printf '%s\n' "$OLD_ENV_IP"
        sed -nE 's/.*<address>([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)<\/address>.*/\1/p' "$XML_FILE"
        sed -nE 's/^[[:space:]]*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)[[:space:]]*=[[:space:]]*RevPi A.*/\1/p' "$XML_FILE"
    } | sort -u | grep -v -x "${PI5_IP:-__no_pi5_ip__}"
)

backup_file "$XML_FILE"
for old_ip in "${XML_REVPI_IPS[@]}"; do
    [ -n "$old_ip" ] || continue
    sed -i "s#${old_ip//./\\.}#$REVPI_NEW_IP#g" "$XML_FILE"
done

# 3) Chuẩn hóa các fallback runtime trong repository. Logic vẫn giữ nguyên:
# tất cả tiếp tục ưu tiên biến môi trường REVPI_A_HOST.
replace_with_sed "$WS/scripts/start_all.sh" \
    "s#REVPI_A_HOST:-[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+#REVPI_A_HOST:-$REVPI_NEW_IP#g"
replace_with_sed "$WS/restart_revpi_rs485.sh" \
    "s#REVPI_A_HOST:-[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+#REVPI_A_HOST:-$REVPI_NEW_IP#g"
replace_with_sed "$WS/src/system_feed_cartridge/scripts/cartridge_gui_web.py" \
    "s#os\.environ\.get\\(\"REVPI_A_HOST\", \"[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+\"\\)#os.environ.get(\"REVPI_A_HOST\", \"$REVPI_NEW_IP\")#g"
replace_with_sed "$DEPLOY_SCRIPT" \
    "s#REVPI_A_HOST:-[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+#REVPI_A_HOST:-$REVPI_NEW_IP#g"

# 4) Đây là file mà shortcut Desktop hiện đang chạy. Chỉ thay ba giá trị IP,
# không thay command, thứ tự node hay SSH options đang hoạt động.
if [ -f "$DESKTOP_START_ALL" ]; then
    replace_with_sed "$DESKTOP_START_ALL" \
        "s#^(REVPI_HOST=\"\\$\\{REVPI_HOST:-)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(\\}\")#\\1$REVPI_NEW_IP\\2#;
         s#^(LOADCELL_HOST=\"\\$\\{LOADCELL_HOST:-)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(\\}\")#\\1$REVPI_NEW_IP\\2#;
         s#(ssh pi@)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+#\\1$REVPI_NEW_IP#g"
fi

echo ""
echo "✅ Đã cập nhật cấu hình local."
echo "📦 Backup trước khi sửa: $BACKUP_DIR"

# Check chính thức của workspace.
if ! WS="$WS" bash "$WS/scripts/check_revpi_ip_sync.sh"; then
    echo "❌ Kiểm tra đồng bộ env/FastDDS thất bại."
    exit 1
fi

# 5) RevPi cũng cần cùng FastDDS profile để discovery hai chiều. Chỉ copy config,
# không build package và không thay đổi logic/node trên RevPi.
if [ "$LOCAL_ONLY" -eq 0 ]; then
    echo ""
    echo "📡 Đồng bộ ROS env + FastDDS sang ${REVPI_USER}@${REVPI_NEW_IP}..."
    SSH_OPTS=(
        -o BatchMode=yes
        -o ConnectTimeout=5
        -o StrictHostKeyChecking=accept-new
        -o UserKnownHostsFile=/dev/null
        -o LogLevel=ERROR
    )
    if ping -c 1 -W 2 "$REVPI_NEW_IP" >/dev/null 2>&1 \
        && scp "${SSH_OPTS[@]}" "$ENV_FILE" "$XML_FILE" \
            "${REVPI_USER}@${REVPI_NEW_IP}:/home/${REVPI_USER}/"; then
        echo "✅ RevPi đã nhận ros2_env.sh và fastdds_peers.xml."
    else
        echo "⚠️  Không thể tự đồng bộ sang RevPi (ping/SSH key chưa sẵn sàng)."
        echo "   Cấu hình local đã đúng. Khi SSH hoạt động, chạy lại file này."
        exit 3
    fi
fi

echo ""
echo "🎉 Hoàn tất. Bây giờ chạy shortcut “Start All ROS2”."
