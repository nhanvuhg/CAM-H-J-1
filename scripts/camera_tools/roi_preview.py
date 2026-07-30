#!/usr/bin/env python3
"""Ve ROI tu YAML len anh de duyet truoc khi dua vao code.

Neu camera dang chay, script con lay bbox that tu YOLO node ve chong len, va
dem xem moi ROI om duoc bao nhieu cartridge -- do chinh la phep tinh
vision_decision_node se lam. ROI dung hay sai nhin phat biet ngay.

Dung:
    python3 roi_preview.py --cam cam0
    python3 roi_preview.py --cam cam1 --no-live      # chi ve ROI, khong can ROS
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml

SECTION = {'cam0': 'input_tray', 'cam1': 'output_tray'}
BOXES_TOPIC = {'cam0': '/cam0HP/yolo/bounding_boxes', 'cam1': '/cam1HP/yolo/bounding_boxes'}
CARTRIDGE_CLASS = '1'   # best.onnx: {0: tray, 1: cartridge, 2: cartridgefall}


def point_in_quad(x, y, quad):
    """Cung phep test voi ROIQuad::contains() ben C++ (cross-product 4 canh)."""
    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    signs = []
    for i in range(4):
        signs.append(cross(quad[i], quad[(i + 1) % 4], (x, y)))
    return all(s >= 0 for s in signs) or all(s <= 0 for s in signs)


def fetch_live_boxes(cam, timeout=8.0):
    """Lay 1 frame bbox tu YOLO node. Tra [] neu ROS/camera khong chay."""
    try:
        import rclpy
        from vision_msgs.msg import Detection2DArray
    except Exception as e:
        print(f'  (bo qua live: {e})')
        return []
    got = []
    try:
        rclpy.init()
        node = rclpy.create_node('roi_preview')

        def cb(msg):
            if not got:
                for d in msg.detections:
                    h = d.results[0].hypothesis
                    got.append((h.class_id, h.score,
                                d.bbox.center.position.x, d.bbox.center.position.y,
                                d.bbox.size_x, d.bbox.size_y))
                got.append(None)   # sentinel: da nhan 1 frame
        node.create_subscription(Detection2DArray, BOXES_TOPIC[cam], cb, 10)
        t0 = time.time()
        while not got and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
        rclpy.shutdown()
    except Exception as e:
        print(f'  (bo qua live: {e})')
        return []
    return [g for g in got if g is not None]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', choices=['cam0', 'cam1'], required=True)
    ap.add_argument('--yaml', default=None)
    ap.add_argument('--image', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--no-live', action='store_true', help='khong lay bbox tu YOLO')
    args = ap.parse_args()

    ypath = args.yaml or os.path.expanduser(f'~/Pictures/roi/{args.cam}_roi.yaml')
    ipath = args.image or os.path.expanduser(f'~/Pictures/roi/{args.cam}.png')
    for p in (ypath, ipath):
        if not os.path.exists(p):
            print(f'[LOI] khong thay {p}')
            return 1

    with open(ypath) as f:
        cfg = yaml.safe_load(f)
    img = cv2.imread(ipath)
    h, w = img.shape[:2]

    ref_w, ref_h = cfg.get('ref_width', w), cfg.get('ref_height', h)
    rois = cfg.get(SECTION[args.cam], {}) or {}
    if not rois:
        print(f'[LOI] {ypath} khong co section "{SECTION[args.cam]}"')
        return 1

    # Anh xem thu phai cung he voi luc cham, neu khong ROI ve ra se lech.
    sx, sy = w / ref_w, h / ref_h
    if (sx, sy) != (1.0, 1.0):
        print(f'[CANH BAO] anh {w}x{h} khac ref {ref_w}x{ref_h} -> scale ROI {sx:.3f},{sy:.3f}')

    boxes = [] if args.no_live else fetch_live_boxes(args.cam)
    carts = [b for b in boxes if b[0] == CARTRIDGE_CLASS]
    print(f'ROI: {len(rois)} | bbox live: {len(boxes)} (cartridge {len(carts)})')

    # bbox truoc, ROI sau -> ROI luon nam tren, khong bi che
    for cid, score, cx, cy, bw, bh in boxes:
        col = (0, 200, 255) if cid == CARTRIDGE_CLASS else (140, 140, 140)
        cv2.rectangle(img, (int(cx - bw / 2), int(cy - bh / 2)),
                      (int(cx + bw / 2), int(cy + bh / 2)), col, 1)
        cv2.circle(img, (int(cx), int(cy)), 2, col, -1)

    quads = {}
    for name, pts in sorted(rois.items()):
        quad = [(int(x * sx), int(y * sy)) for x, y in pts]
        quads[name] = quad
        n = sum(1 for c in carts if point_in_quad(c[2], c[3], quad))
        col = (0, 255, 0) if n else (80, 80, 220)
        cv2.polylines(img, [np.array(quad, np.int32)], True, col, 2)
        label = f'{name}:{n}'
        ox, oy = quad[0]
        cv2.rectangle(img, (ox, oy - 12), (ox + 8 * len(label), oy), col, -1)
        cv2.putText(img, label, (ox + 2, oy - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        print(f'  {name:8s} {n:2d} cartridge')

    # "outer" bao trum cac row nen khong duoc cong don theo ROI — dem theo
    # tung cartridge: roi vao it nhat 1 ROI hay khong.
    outside = sum(1 for c in carts
                  if not any(point_in_quad(c[2], c[3], q) for q in quads.values()))
    print(f'Cartridge ngoai moi ROI: {outside}/{len(carts)}')
    if carts and outside == len(carts):
        print('  [NGHI NGO] KHONG cartridge nao roi vao ROI -> nhieu kha nang sai he toa do.')

    out = args.out or os.path.expanduser(f'~/Pictures/roi/{args.cam}_preview.png')
    cv2.imwrite(out, img)
    print(f'-> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
