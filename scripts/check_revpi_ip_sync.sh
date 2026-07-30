#!/bin/bash
# Sanity check: $REVPI_A_HOST (ros2_env.sh) phải khớp <address> trong
# fastdds_peers.xml. FastDDS không support env-var trong locator address
# nên file XML phải sửa tay khi đổi IP — script này nhắc nếu lệch.
set -u

WS="${WS:-/home/pi/ros2_ws}"
ENV_FILE="$WS/ros2_env.sh"
XML_FILE="$WS/fastdds_peers.xml"

# shellcheck source=/dev/null
source "$ENV_FILE"

if [ -z "${REVPI_A_HOST:-}" ]; then
    echo "❌ REVPI_A_HOST chưa được set trong $ENV_FILE"
    exit 1
fi

# Lấy IP Pi 5 từ phần chú thích của XML rồi loại nó khỏi danh sách RevPi.
# Không cố định subnet để check vẫn đúng nếu DHCP chuyển RevPi sang mạng khác.
pi5_ip=$(sed -nE \
    's/^[[:space:]]*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)[[:space:]]*=[[:space:]]*Raspberry Pi 5.*/\1/p' \
    "$XML_FILE" | head -n1)

xml_ips=$(grep -oE '<address>[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+</address>' "$XML_FILE" \
            | sed -E 's/<\/?address>//g' \
            | grep -v -x "${pi5_ip:-__no_pi5_ip__}" \
            | sort -u)

if [ -z "$xml_ips" ]; then
    echo "⚠️  Không tìm thấy IP RevPi A trong $XML_FILE"
    exit 1
fi

mismatch=0
for ip in $xml_ips; do
    if [ "$ip" != "$REVPI_A_HOST" ]; then
        echo "❌ Mismatch: XML có $ip nhưng REVPI_A_HOST=$REVPI_A_HOST"
        mismatch=1
    fi
done

if [ $mismatch -eq 0 ]; then
    echo "✅ REVPI_A_HOST=$REVPI_A_HOST khớp với fastdds_peers.xml"
    exit 0
fi

echo ""
echo "→ Chạy file Desktop/update_revpi_ip.sh để đồng bộ tự động."
exit 1
