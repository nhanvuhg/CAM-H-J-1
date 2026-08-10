#!/usr/bin/env bash
# Stage and patch RevPi A error logging. Read-only unless --apply is explicit.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_HOST="pi@172.16.11.31"
REMOTE_TARGET="/home/pi/ros2_jazzy/src/fill_hp/fill_hp/web_hp.py"
REMOTE_PRODUCTION_LOG_TARGET="/home/pi/ros2_jazzy/src/fill_hp/fill_hp/production_log.py"
REMOTE_BACKUP_DIR="/home/pi/cartridge_fill_logs/web_hp_backups"
REMOTE_WORKSPACE="/home/pi/ros2_jazzy"
APPLY=0

usage() {
    printf '%s\n' \
        "Usage: $0 [--host user@host] [--target /absolute/web_hp.py] [--production-log-target /absolute/production_log.py] [--workspace /absolute/path] [--backup-dir /absolute/path] [--apply]" \
        "" \
        "Default: inspect only. --apply patches atomically and creates a backup." \
        "This script never restarts web_hp or any ROS/systemd service."
}

while (($#)); do
    case "$1" in
        --host)
            REMOTE_HOST="${2:?missing value for --host}"
            shift 2
            ;;
        --target)
            REMOTE_TARGET="${2:?missing value for --target}"
            shift 2
            ;;
        --production-log-target)
            REMOTE_PRODUCTION_LOG_TARGET="${2:?missing value for --production-log-target}"
            shift 2
            ;;
        --backup-dir)
            REMOTE_BACKUP_DIR="${2:?missing value for --backup-dir}"
            shift 2
            ;;
        --workspace)
            REMOTE_WORKSPACE="${2:?missing value for --workspace}"
            shift 2
            ;;
        --apply)
            APPLY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

# These values are passed through a remote shell. Restrict them to the simple
# host/path forms used by this machine instead of attempting fragile escaping.
if [[ ! "$REMOTE_HOST" =~ ^[A-Za-z0-9_.-]+@[A-Za-z0-9_.:-]+$ ]]; then
    printf 'Unsafe --host value: %s\n' "$REMOTE_HOST" >&2
    exit 2
fi
if [[ ! "$REMOTE_TARGET" =~ ^/[A-Za-z0-9_./-]+$ ]] ||
   [[ ! "$REMOTE_PRODUCTION_LOG_TARGET" =~ ^/[A-Za-z0-9_./-]+$ ]] ||
   [[ ! "$REMOTE_BACKUP_DIR" =~ ^/[A-Za-z0-9_./-]+$ ]] ||
   [[ ! "$REMOTE_WORKSPACE" =~ ^/[A-Za-z0-9_./-]+$ ]]; then
    printf 'Target and backup directory must be simple absolute paths.\n' >&2
    exit 2
fi

printf 'RevPi:  %s\nWeb target: %s\nProduction log: %s\n' \
    "$REMOTE_HOST" "$REMOTE_TARGET" "$REMOTE_PRODUCTION_LOG_TARGET"

if ((APPLY == 0)); then
    printf '%s\n' 'CHECK ONLY — no remote files will be changed.'
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
        "test -f '$REMOTE_TARGET' && test -f '$REMOTE_PRODUCTION_LOG_TARGET' && \
         sha256sum '$REMOTE_TARGET' '$REMOTE_PRODUCTION_LOG_TARGET' && \
         grep -c 'CAM_HJ1_EXTERNAL_EVENT_LOG_V1' '$REMOTE_TARGET' || true"
    printf '%s\n' 'Run again with --apply to install. No service will be restarted.'
    exit 0
fi

REMOTE_STAGE="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
    "mktemp -d /tmp/cam-hj1-event-log.XXXXXX")"
if [[ ! "$REMOTE_STAGE" =~ ^/tmp/cam-hj1-event-log\.[A-Za-z0-9]+$ ]]; then
    printf 'Unexpected remote staging path: %s\n' "$REMOTE_STAGE" >&2
    exit 1
fi

cleanup() {
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
        "rm -rf -- '$REMOTE_STAGE'" >/dev/null 2>&1 || true
}
trap cleanup EXIT

scp -q \
    "$SCRIPT_DIR/external_event_log.py" \
    "$SCRIPT_DIR/patch_web_hp.py" \
    "$SCRIPT_DIR/patch_production_log.py" \
    "$REMOTE_HOST:$REMOTE_STAGE/"

# Parse the complete proposed production_log.py before changing any source.
ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
    "python3 '$REMOTE_STAGE/patch_production_log.py' \
        --target '$REMOTE_PRODUCTION_LOG_TARGET' \
        --backup-dir '$REMOTE_BACKUP_DIR'"

ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
    "python3 '$REMOTE_STAGE/patch_web_hp.py' \
        --target '$REMOTE_TARGET' \
        --module-source '$REMOTE_STAGE/external_event_log.py' \
        --backup-dir '$REMOTE_BACKUP_DIR' \
        --module-only \
        --apply"

# Stop Errors are durable only for ERROR incidents in AUTO/AI. Manual/JOG and
# WARN remain runtime diagnostics and are not appended to production history.
ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
    "python3 '$REMOTE_STAGE/patch_production_log.py' \
        --target '$REMOTE_PRODUCTION_LOG_TARGET' \
        --backup-dir '$REMOTE_BACKUP_DIR' \
        --apply"

# Install the companion package before web_hp.py starts importing it. A build
# failure leaves the currently running and on-disk web bridge unpatched.
ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
    "bash -lc 'cd \"$REMOTE_WORKSPACE\" && \
        if [ -f /opt/ros/jazzy/setup.bash ]; then \
            source /opt/ros/jazzy/setup.bash; \
        elif [ -f \"$REMOTE_WORKSPACE/install/setup.bash\" ]; then \
            source \"$REMOTE_WORKSPACE/install/setup.bash\"; \
        else \
            printf \"No ROS setup found for fill_hp build\\n\" >&2; exit 1; \
        fi && \
        colcon build --packages-select fill_hp --symlink-install && \
        source install/setup.bash && \
        python3 -c \"from fill_hp import external_event_log as m; assert hasattr(m, \\\"ExternalEventLog\\\")\"'"

# The installed import is now known-good; only then patch the live source.
ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE_HOST" \
    "python3 '$REMOTE_STAGE/patch_web_hp.py' \
        --target '$REMOTE_TARGET' \
        --module-source '$REMOTE_STAGE/external_event_log.py' \
        --backup-dir '$REMOTE_BACKUP_DIR' \
        --apply"

printf '%s\n' \
    'Companion module and AUTO/AI Stop Errors policy built and verified.' \
    'No process was restarted. Restart web_hp later in a controlled maintenance window.'
