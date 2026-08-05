#!/usr/bin/env bash
set -uo pipefail

# Load the patched IMX477 driver only after the external V3Link receivers have
# had time to settle.  A failed sensor probe is cleaned up before retrying, so
# the reset GPIO and tegracam resources are not leaked until the next reboot.

CAMH_CAMERA_PATCH_VERSION="${CAMH_CAMERA_PATCH_VERSION:-2.0.6-camhj1-cleanup1}"
CAMH_CAMERA_SETTLE_SEC="${CAMH_CAMERA_SETTLE_SEC:-10}"
CAMH_CAMERA_RETRY_SEC="${CAMH_CAMERA_RETRY_SEC:-5}"
CAMH_CAMERA_ATTEMPTS="${CAMH_CAMERA_ATTEMPTS:-1}"
CAMH_CAMERA_LOCK="${CAMH_CAMERA_LOCK:-/run/lock/cam-h-j-1-imx477.lock}"
CAMH_CAMERA_STATUS="${CAMH_CAMERA_STATUS:-/run/cam-h-j-1-imx477.status}"

usage() {
    echo "Usage: $0 [--attempts N] [--settle SEC] [--retry SEC]" >&2
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --attempts)
            [ "$#" -ge 2 ] || { usage; exit 64; }
            CAMH_CAMERA_ATTEMPTS="$2"
            shift 2
            ;;
        --settle)
            [ "$#" -ge 2 ] || { usage; exit 64; }
            CAMH_CAMERA_SETTLE_SEC="$2"
            shift 2
            ;;
        --retry)
            [ "$#" -ge 2 ] || { usage; exit 64; }
            CAMH_CAMERA_RETRY_SEC="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage
            exit 64
            ;;
    esac
done

case "$CAMH_CAMERA_ATTEMPTS:$CAMH_CAMERA_SETTLE_SEC:$CAMH_CAMERA_RETRY_SEC" in
    *[!0-9:]*|0:*)
        echo "attempts must be positive; delays must be non-negative integers" >&2
        exit 64
        ;;
esac

write_status() {
    local status_text="$1"
    printf '%s %s\n' "$(date --iso-8601=seconds)" "$status_text" \
        > "$CAMH_CAMERA_STATUS" 2>/dev/null || true
    echo "[$(date --iso-8601=seconds)] $status_text"
}

camera_devices_ready() {
    [ -c /dev/video0 ] && [ -c /dev/video1 ]
}

camera_driver_loaded() {
    [ -d /sys/module/nv_imx477 ]
}

camera_driver_retry_safe() {
    [ -r /sys/module/nv_imx477/version ] &&
        [ "$(cat /sys/module/nv_imx477/version)" = "$CAMH_CAMERA_PATCH_VERSION" ]
}

camera_device_holders() {
    local camera_device
    for camera_device in /dev/video0 /dev/video1; do
        [ -e "$camera_device" ] || continue
        fuser "$camera_device" 2>/dev/null || true
    done | tr ' ' '\n' | sed '/^$/d' | sort -nu
}

if [ "$(id -u)" -ne 0 ]; then
    echo "This helper must run as root (use sudo -n)." >&2
    exit 77
fi

mkdir -p "$(dirname "$CAMH_CAMERA_LOCK")"
exec 9>"$CAMH_CAMERA_LOCK"
if ! flock -w 60 9; then
    write_status "FAILED reason=driver_init_lock_timeout"
    exit 75
fi

if camera_devices_ready; then
    write_status "READY video0=1 video1=1"
    exit 0
fi

camera_attempt=1
while [ "$camera_attempt" -le "$CAMH_CAMERA_ATTEMPTS" ]; do
    if camera_driver_loaded; then
        camera_holders="$(camera_device_holders)"
        if [ -n "$camera_holders" ]; then
            write_status "HELD partial_probe=1 holders=$(echo "$camera_holders" | paste -sd, -)"
            exit 73
        fi
        if ! camera_driver_retry_safe; then
            loaded_version="$(cat /sys/module/nv_imx477/version 2>/dev/null || echo unknown)"
            write_status "FAILED reason=unsafe_loaded_driver version=$loaded_version"
            exit 78
        fi
        if ! modprobe -r nv_imx477; then
            write_status "FAILED reason=driver_unload"
            exit 1
        fi
        udevadm settle --timeout=10 2>/dev/null || true
    fi

    if [ "$camera_attempt" -eq 1 ]; then
        camera_delay="$CAMH_CAMERA_SETTLE_SEC"
    else
        camera_delay="$CAMH_CAMERA_RETRY_SEC"
    fi
    write_status "WAITING attempt=$camera_attempt/$CAMH_CAMERA_ATTEMPTS delay_s=$camera_delay"
    sleep "$camera_delay"

    write_status "LOADING attempt=$camera_attempt/$CAMH_CAMERA_ATTEMPTS module=nv_imx477"
    if modprobe nv_imx477; then
        udevadm settle --timeout=10 2>/dev/null || true
        sleep 2
    fi

    if camera_devices_ready; then
        write_status "READY video0=1 video1=1 attempt=$camera_attempt"
        exit 0
    fi

    # Leave a failed partial probe unloaded.  This is safe only with the local
    # cleanup patch, which unregisters tegracam after an I2C probe failure.
    if camera_driver_loaded && camera_driver_retry_safe; then
        camera_holders="$(camera_device_holders)"
        if [ -z "$camera_holders" ]; then
            modprobe -r nv_imx477 2>/dev/null || true
            udevadm settle --timeout=10 2>/dev/null || true
        fi
    fi

    write_status "RETRY missing_video_devices=1 attempt=$camera_attempt/$CAMH_CAMERA_ATTEMPTS"
    camera_attempt=$((camera_attempt + 1))
done

write_status "FAILED reason=cameras_not_ready attempts=$CAMH_CAMERA_ATTEMPTS"
exit 1
