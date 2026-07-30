#!/usr/bin/env python3
"""
ROI Picker — multi-ROI version. Click 4 corners per ROI, lặp qua nhiều ROI có
tên (vd: TRAY + ROW1..ROW5 = 6 ROI × 4 corners = 24 clicks).

Output coords ở production 1280×720 (matching cam0HP/yolo/bounding_boxes).

Usage:
    python3 roi_picker.py [/path/to/image.png]
    # No arg → auto pick latest ~/Pictures/roi/*.png

Default ROI sequence: TRAY, ROW1, ROW2, ROW3, ROW4, ROW5
    Customize: ROI_NAMES=TRAY,R1,R2,R3,R4,R5,R6 python3 roi_picker.py

Controls:
    Left click   → place corner (4 corners per ROI, auto-advance)
    R            → reset CURRENT ROI clicks
    B            → back to previous ROI
    S            → skip current ROI (leaves it empty)
    Q / ESC      → quit early
    Enter        → confirm (after all ROIs done) or skip-to-next
"""
import cv2
import sys
import os
import glob

PROD_W, PROD_H = 1280, 720

DEFAULT_ROI_NAMES = ["TRAY", "ROW1", "ROW2", "ROW3", "ROW4", "ROW5"]
ROI_NAMES = os.environ.get("ROI_NAMES", "").strip()
ROI_NAMES = [s.strip() for s in ROI_NAMES.split(",") if s.strip()] or DEFAULT_ROI_NAMES

# State
current_roi_idx = 0
roi_corners = {name: [] for name in ROI_NAMES}
draw_img = None
base_img = None

# Distinct colors per ROI (BGR)
PALETTE = [
    (0, 255, 0),     # green - TRAY
    (0, 200, 255),   # orange - ROW1
    (255, 100, 100), # blue  - ROW2
    (255, 255, 0),   # cyan  - ROW3
    (200, 100, 255), # pink  - ROW4
    (100, 255, 255), # yellow- ROW5
    (255, 0, 255),   # magenta
    (0, 100, 255),   # red-orange
]


def color_for(idx):
    return PALETTE[idx % len(PALETTE)]


def redraw_all():
    """Render base image + all stored ROIs."""
    global draw_img
    draw_img = base_img.copy()
    for idx, name in enumerate(ROI_NAMES):
        color = color_for(idx)
        pts = roi_corners[name]
        for i, (x, y) in enumerate(pts):
            cv2.circle(draw_img, (x, y), 6, color, -1)
            label = f"{name[0:3]}{i+1}"
            cv2.putText(draw_img, label, (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        # connect lines
        n = len(pts)
        if n >= 2:
            for i in range(n - 1):
                cv2.line(draw_img, pts[i], pts[i+1], color, 2)
        if n == 4:
            cv2.line(draw_img, pts[3], pts[0], color, 2)

    # Header overlay
    if current_roi_idx < len(ROI_NAMES):
        name = ROI_NAMES[current_roi_idx]
        done = len(roi_corners[name])
        header = f"{current_roi_idx+1}/{len(ROI_NAMES)}  {name}: {done}/4 corners"
        color = color_for(current_roi_idx)
    else:
        header = "All ROIs picked — press Enter to confirm"
        color = (0, 255, 0)
    cv2.rectangle(draw_img, (0, 0), (PROD_W, 30), (0, 0, 0), -1)
    cv2.putText(draw_img, header, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                color, 2)

    cv2.imshow('ROI Picker', draw_img)


def on_mouse(event, x, y, flags, param):
    global current_roi_idx
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    if current_roi_idx >= len(ROI_NAMES):
        return
    name = ROI_NAMES[current_roi_idx]
    if len(roi_corners[name]) >= 4:
        return
    roi_corners[name].append((x, y))
    print(f"  [{name}] Corner {len(roi_corners[name])}: ({x}, {y})")
    if len(roi_corners[name]) == 4:
        current_roi_idx += 1
        if current_roi_idx < len(ROI_NAMES):
            next_name = ROI_NAMES[current_roi_idx]
            print(f"→ Next: {next_name}")
        else:
            print("→ All ROIs done. Press Enter to confirm, B to revise.")
    redraw_all()


def main():
    global base_img, current_roi_idx

    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        roi_pics = sorted(glob.glob(os.path.expanduser('~/Pictures/roi/*.png')),
                          key=os.path.getmtime)
        if roi_pics:
            path = roi_pics[-1]
            print(f"Using latest ROI capture: {path}")
        else:
            train_pics = sorted(glob.glob(os.path.expanduser('~/Pictures/cam_*.png')),
                                key=os.path.getmtime)
            if not train_pics:
                print("❌ No images. Run camera_roi.py first.")
                sys.exit(1)
            path = train_pics[-1]
            print(f"⚠️  No ROI capture. Using training image (will resize): {path}")

    img_orig = cv2.imread(path)
    if img_orig is None:
        print(f"❌ Cannot read {path}")
        sys.exit(1)

    h, w = img_orig.shape[:2]
    print(f"Source: {w}×{h}")
    if (w, h) != (PROD_W, PROD_H):
        print(f"⚠️  Resizing to production {PROD_W}×{PROD_H}")
        base_img = cv2.resize(img_orig, (PROD_W, PROD_H))
    else:
        base_img = img_orig.copy()

    print()
    print(f"ROI sequence ({len(ROI_NAMES)} ROIs × 4 corners "
          f"= {len(ROI_NAMES)*4} clicks):")
    for i, n in enumerate(ROI_NAMES):
        print(f"  {i+1}. {n}")
    print()
    print("Keys: R=reset current  B=back  S=skip current  "
          "Q/ESC=quit  Enter=confirm/next")
    print()

    cv2.namedWindow('ROI Picker', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('ROI Picker', PROD_W, PROD_H)
    cv2.setMouseCallback('ROI Picker', on_mouse)
    redraw_all()

    while True:
        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord('q')):
            print("Cancelled.")
            cv2.destroyAllWindows()
            return
        if key == ord('r') and current_roi_idx < len(ROI_NAMES):
            name = ROI_NAMES[current_roi_idx]
            print(f"Reset {name}.")
            roi_corners[name].clear()
            redraw_all()
        if key == ord('b') and current_roi_idx > 0:
            current_roi_idx -= 1
            name = ROI_NAMES[current_roi_idx]
            roi_corners[name].clear()
            print(f"← Back to {name}.")
            redraw_all()
        if key == ord('s') and current_roi_idx < len(ROI_NAMES):
            name = ROI_NAMES[current_roi_idx]
            print(f"Skipped {name}.")
            current_roi_idx += 1
            redraw_all()
        if key in (13, 10):
            if current_roi_idx >= len(ROI_NAMES):
                break  # all done, confirm
            # else: ignore Enter mid-pick

    cv2.destroyAllWindows()

    # Report
    print()
    print("=" * 72)
    print(f"ROI corners — production 1280×720 (matches Detection2D bbox space)")
    print("=" * 72)

    for name in ROI_NAMES:
        pts = roi_corners[name]
        print(f"\n--- {name} ({len(pts)}/4 corners) ---")
        if not pts:
            print("  (skipped/empty)")
            continue
        for i, (x, y) in enumerate(pts):
            print(f"  P{i+1}: ({x}, {y})")

    # C++ paste
    print()
    print("─── C++ paste block ──────────────────────────────────────────────")
    print("// Tray + rows (paste vào vision_decision_node.cpp / robot_logic_node.cpp)")
    for name in ROI_NAMES:
        pts = roi_corners[name]
        if len(pts) != 4:
            print(f"// {name} — incomplete ({len(pts)} corners), bỏ qua")
            continue
        print(f"std::vector<std::pair<int,int>> {name.lower()}_corners = {{")
        for x, y in pts:
            print(f"    {{{x}, {y}}},")
        print("};")

    print()
    print("─── Axis-aligned bbox per ROI ───────────────────────────────────")
    for name in ROI_NAMES:
        pts = roi_corners[name]
        if len(pts) != 4:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        print(f"  {name:8s}  min_x={min(xs):4d}  max_x={max(xs):4d}  "
              f"min_y={min(ys):4d}  max_y={max(ys):4d}")

    # Optional: save to JSON for reuse
    try:
        import json
        out_path = os.path.expanduser(f'~/Pictures/roi/last_picks.json')
        with open(out_path, 'w') as f:
            json.dump({n: roi_corners[n] for n in ROI_NAMES}, f, indent=2)
        print(f"\n💾 Saved to {out_path}")
    except Exception as e:
        print(f"(JSON save failed: {e})")


if __name__ == '__main__':
    main()
