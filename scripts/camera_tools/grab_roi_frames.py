#!/usr/bin/env python3
"""Chup truc tiep 2 camera de cham ROI, khong can ROS/launch.

Dung cung full-sensor mode 4056x3040 (4:3) cua camera runtime va xuat
640x480. ROI cham tren anh nay cung kich thuoc va cung ti le voi bbox YOLO.
Camera phai dang ranh; dung camera launch truoc khi chay script.

Dung:
    python3 grab_roi_frames.py                 # ca 2 cam
    python3 grab_roi_frames.py --cam cam0      # rieng cam0
    python3 grab_roi_frames.py --out ~/roi     # thu muc khac
"""
import argparse
import os
import subprocess
import sys
import time

import cv2
import numpy as np

CAMERA_IDS = {'cam0': 0, 'cam1': 1}
WIDTH, HEIGHT = 640, 480
SENSOR_MODE = '4056:3040:12:P'


def capture_direct(camera_id, settle_ms, timeout):
    """Tra anh BGR chup truc tiep tu rpicam-still."""
    cmd = [
        'rpicam-still', '--camera', str(camera_id),
        '-t', str(settle_ms), '--nopreview',
        '--width', str(WIDTH), '--height', str(HEIGHT),
        '--mode', SENSOR_MODE,
        '--denoise', 'cdn_off',
        '--shutter', '12000',
        '--awbgains', '3.33,1.55',
        '--encoding', 'png', '-o', '-',
    ]
    env = os.environ.copy()
    env.setdefault(
        'LIBCAMERA_RPI_CONFIG_FILE',
        '/home/pi/ros2_ws/pisp_camera_config.yaml')
    try:
        proc = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout, env=env, check=False)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f'rpicam-still timeout sau {timeout:.0f}s') from e
    if proc.returncode != 0:
        detail = proc.stderr.decode(errors='replace').strip().splitlines()
        raise RuntimeError(detail[-1] if detail else f'rpicam-still exit {proc.returncode}')
    img = cv2.imdecode(np.frombuffer(proc.stdout, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError('rpicam-still khong tra ve anh PNG hop le')
    if img.shape[1] != WIDTH or img.shape[0] != HEIGHT:
        raise RuntimeError(f'kich thuoc sai: {img.shape[1]}x{img.shape[0]}')
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', choices=list(CAMERA_IDS) + ['all'], default='all')
    ap.add_argument('--out', default=os.path.expanduser('~/Pictures/roi'))
    ap.add_argument('--timeout', type=float, default=15.0,
                    help='timeout cho moi camera')
    ap.add_argument('--settle-ms', type=int, default=1500,
                    help='thoi gian camera on dinh exposure truoc khi chup')
    args = ap.parse_args()

    wanted = list(CAMERA_IDS) if args.cam == 'all' else [args.cam]
    os.makedirs(args.out, exist_ok=True)

    rc = 0
    for name in wanted:
        camera_id = CAMERA_IDS[name]
        print(f'Chup truc tiep {name} (camera {camera_id}) ...', flush=True)
        try:
            img = capture_direct(camera_id, args.settle_ms, args.timeout)
        except Exception as e:
            print(f'  [LOI] {name}: {e}')
            print('        -> dam bao camera launch/rpicam-vid da dung va camera khong bi process khac giu')
            rc = 1
            continue
        path = os.path.join(args.out, f'{name}.png')
        cv2.imwrite(path, img)
        # Ban chinh (cam0.png) la thu roi_pick.py doc; ban timestamp de cac lan
        # chup thu cong khong de mat anh cu.
        stamp = time.strftime('%Y%m%d_%H%M%S')
        archive = os.path.join(args.out, f'{name}_{stamp}.png')
        cv2.imwrite(archive, img)
        print(f'  {name}: {WIDTH}x{HEIGHT} bgr8 -> {path}')
        print(f'        (ban luu: {archive})')

    if rc == 0:
        print('\nBuoc tiep: python3 roi_pick.py --cam cam0   (roi --cam cam1)')
    return rc


if __name__ == '__main__':
    sys.exit(main())
