#!/usr/bin/env python3
"""
ROI Picker (web version) — click corners trên browser, KHÔNG cần display Pi.
Phục vụ ảnh ROI capture, render canvas, dùng cho SSH-only workflow.

Chạy:
    python3 /home/pi/roi_picker_web.py [/path/to/image.png]

Mở browser ở máy bạn: http://<pi-ip>:8090
(Nếu SSH có port forward: -L 8090:localhost:8090 → http://localhost:8090)

Default ROI sequence: TRAY, ROW1..ROW5 (6 × 4 = 24 corners)
Custom: ROI_NAMES=TRAY,R1,R2,R3,R4,R5 python3 roi_picker_web.py
"""
import cv2
import sys
import os
import glob
import json
import socket
from flask import Flask, request, jsonify, send_file, Response

PROD_W, PROD_H = 1280, 720
PORT = 8090

DEFAULT_ROI_NAMES = ["TRAY", "ROW1", "ROW2", "ROW3", "ROW4", "ROW5"]
ROI_NAMES = os.environ.get("ROI_NAMES", "").strip()
ROI_NAMES = [s.strip() for s in ROI_NAMES.split(",") if s.strip()] or DEFAULT_ROI_NAMES

app = Flask(__name__)
prepared_image_path = None  # Will be set in main()


def prepare_image(path):
    """Read source image, resize to production 1280x720 if needed, save tmp."""
    img = cv2.imread(path)
    if img is None:
        print(f"❌ Cannot read {path}")
        sys.exit(1)
    h, w = img.shape[:2]
    print(f"Source: {path}  {w}×{h}")
    if (w, h) != (PROD_W, PROD_H):
        print(f"⚠️  Resizing to {PROD_W}×{PROD_H}")
        img = cv2.resize(img, (PROD_W, PROD_H))
    out = '/tmp/roi_picker_image.png'
    cv2.imwrite(out, img)
    return out


HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>ROI Picker (web)</title>
<style>
  body { font-family: sans-serif; background: #1e1e1e; color: #eee; margin: 0; padding: 10px; }
  h2 { margin: 0 0 8px 0; }
  #status { padding: 6px 12px; background: #2d2d2d; border-radius: 4px;
            margin-bottom: 8px; font-family: monospace; }
  .row { display: flex; gap: 12px; align-items: flex-start; }
  #canvas-wrap { position: relative; flex: 0 0 auto; border: 1px solid #444; }
  canvas { cursor: crosshair; display: block; }
  #side { flex: 1; min-width: 280px; }
  button { background: #3a3a3a; color: #eee; border: 1px solid #555;
           padding: 6px 14px; margin: 3px; cursor: pointer; border-radius: 3px; }
  button:hover { background: #4a4a4a; }
  button.primary { background: #1a5; border-color: #2c7; }
  button.danger { background: #a31; border-color: #d52; }
  pre { background: #0a0a0a; padding: 10px; overflow: auto; font-size: 11px;
        max-height: 380px; border: 1px solid #333; }
  .roi-list { margin: 8px 0; }
  .roi-item { padding: 4px 8px; margin: 2px 0; border-radius: 3px;
              font-family: monospace; }
  .roi-item.current { background: #2a4a2a; font-weight: bold; }
  .roi-item.done { background: #2a3a2a; color: #afa; }
</style></head>
<body>
<h2>ROI Picker — production 1280×720</h2>
<div id="status">Loading...</div>
<div class="row">
  <div id="canvas-wrap">
    <canvas id="cv" width="1280" height="720"></canvas>
  </div>
  <div id="side">
    <div>
      <button class="primary" onclick="confirmAll()">✓ Confirm &amp; Save</button>
      <button class="danger" onclick="resetCurrent()">Reset current ROI</button>
      <button onclick="back()">← Back</button>
      <button onclick="skip()">Skip →</button>
    </div>
    <div class="roi-list" id="roi-list"></div>
    <pre id="output">Click 4 corners per ROI. Output sẽ hiện ở đây sau Confirm.</pre>
  </div>
</div>
<script>
const ROI_NAMES = {{ roi_names | tojson }};
const PROD_W = 1280, PROD_H = 720;
const PALETTE = ['#00ff00','#ffaa00','#6464ff','#ffff00','#ff64ff','#64ffff',
                 '#ff00ff','#ff6400'];
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
const img = new Image();
let imgLoaded = false;
let cur = 0;
let corners = {};
ROI_NAMES.forEach(n => corners[n] = []);

img.onload = () => { imgLoaded = true; redraw(); };
img.src = '/image?t=' + Date.now();

cv.addEventListener('click', e => {
  if (cur >= ROI_NAMES.length) return;
  const name = ROI_NAMES[cur];
  if (corners[name].length >= 4) return;
  const r = cv.getBoundingClientRect();
  // Canvas internal coords are 1280x720, scale to clicked position
  const x = Math.round((e.clientX - r.left) * cv.width / r.width);
  const y = Math.round((e.clientY - r.top) * cv.height / r.height);
  corners[name].push([x, y]);
  console.log(`${name} corner ${corners[name].length}: (${x}, ${y})`);
  if (corners[name].length === 4) {
    cur++;
  }
  redraw();
});

document.addEventListener('keydown', e => {
  if (e.key === 'r' || e.key === 'R') resetCurrent();
  else if (e.key === 'b' || e.key === 'B') back();
  else if (e.key === 's' || e.key === 'S') skip();
  else if (e.key === 'Enter') confirmAll();
});

function resetCurrent() {
  if (cur < ROI_NAMES.length) {
    corners[ROI_NAMES[cur]] = [];
    redraw();
  }
}
function back() {
  if (cur > 0) {
    cur--;
    corners[ROI_NAMES[cur]] = [];
    redraw();
  }
}
function skip() {
  if (cur < ROI_NAMES.length) {
    cur++;
    redraw();
  }
}

function redraw() {
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, cv.width, cv.height);
  if (imgLoaded) ctx.drawImage(img, 0, 0, cv.width, cv.height);
  // overlay header
  ctx.fillStyle = 'rgba(0,0,0,0.7)';
  ctx.fillRect(0, 0, cv.width, 32);
  let header;
  let headerColor;
  if (cur < ROI_NAMES.length) {
    const name = ROI_NAMES[cur];
    header = `${cur+1}/${ROI_NAMES.length}  ${name}: ${corners[name].length}/4`;
    headerColor = PALETTE[cur % PALETTE.length];
  } else {
    header = 'All ROIs picked — press Enter or click Confirm';
    headerColor = '#0f0';
  }
  ctx.font = 'bold 18px monospace';
  ctx.fillStyle = headerColor;
  ctx.fillText(header, 10, 22);
  // draw all ROIs
  ROI_NAMES.forEach((name, idx) => {
    const color = PALETTE[idx % PALETTE.length];
    const pts = corners[name];
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2;
    pts.forEach((p, i) => {
      ctx.beginPath();
      ctx.arc(p[0], p[1], 6, 0, 2*Math.PI);
      ctx.fill();
      ctx.font = '12px monospace';
      ctx.fillText(`${name.substr(0,3)}${i+1}`, p[0]+9, p[1]-8);
    });
    if (pts.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(pts[0][0], pts[0][1]);
      for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i][0], pts[i][1]);
      if (pts.length === 4) ctx.closePath();
      ctx.stroke();
    }
  });
  updateList();
}

function updateList() {
  const list = document.getElementById('roi-list');
  list.innerHTML = '';
  ROI_NAMES.forEach((name, idx) => {
    const d = document.createElement('div');
    const n = corners[name].length;
    let cls = 'roi-item';
    if (idx === cur) cls += ' current';
    else if (n === 4) cls += ' done';
    d.className = cls;
    d.textContent = `${idx+1}. ${name}  [${n}/4]`;
    list.appendChild(d);
  });
  document.getElementById('status').textContent =
    `ROI ${Math.min(cur+1, ROI_NAMES.length)}/${ROI_NAMES.length} — ` +
    `Total clicks: ${Object.values(corners).reduce((s,a)=>s+a.length,0)}/${ROI_NAMES.length*4}`;
}

function confirmAll() {
  fetch('/submit', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(corners)
  }).then(r => r.text()).then(t => {
    document.getElementById('output').textContent = t;
  });
}
</script>
</body></html>
"""


@app.route('/')
def index():
    from flask import render_template_string
    return render_template_string(HTML, roi_names=ROI_NAMES)


@app.route('/image')
def image():
    return send_file(prepared_image_path, mimetype='image/png')


@app.route('/submit', methods=['POST'])
def submit():
    data = request.json
    lines = []
    lines.append("=" * 72)
    lines.append("ROI corners — production 1280×720")
    lines.append("=" * 72)
    for name in ROI_NAMES:
        pts = data.get(name, [])
        lines.append(f"\n--- {name} ({len(pts)}/4) ---")
        for i, p in enumerate(pts):
            lines.append(f"  P{i+1}: ({p[0]}, {p[1]})")
        if len(pts) == 0:
            lines.append("  (skipped)")

    lines.append("\n─── C++ paste ───────────────────────────────────")
    for name in ROI_NAMES:
        pts = data.get(name, [])
        if len(pts) != 4:
            lines.append(f"// {name} — incomplete")
            continue
        lines.append(f"std::vector<std::pair<int,int>> {name.lower()}_corners = {{")
        for p in pts:
            lines.append(f"    {{{p[0]}, {p[1]}}},")
        lines.append("};")

    lines.append("\n─── Axis-aligned bbox ───────────────────────────")
    for name in ROI_NAMES:
        pts = data.get(name, [])
        if len(pts) != 4:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        lines.append(f"  {name:8s}  min_x={min(xs):4d}  max_x={max(xs):4d}  "
                     f"min_y={min(ys):4d}  max_y={max(ys):4d}")

    out_text = "\n".join(lines)
    # Print to server terminal
    print()
    print(out_text)

    # Save JSON
    out_json = os.path.expanduser('~/Pictures/roi/last_picks.json')
    try:
        os.makedirs(os.path.dirname(out_json), exist_ok=True)
        with open(out_json, 'w') as f:
            json.dump(data, f, indent=2)
        out_text += f"\n\n💾 Saved JSON: {out_json}"
        print(f"💾 Saved JSON: {out_json}")
    except Exception as e:
        out_text += f"\n\n(JSON save failed: {e})"
    return Response(out_text, mimetype='text/plain')


def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def main():
    global prepared_image_path
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        pics = sorted(glob.glob(os.path.expanduser('~/Pictures/roi/*.png')),
                      key=os.path.getmtime)
        if not pics:
            pics = sorted(glob.glob(os.path.expanduser('~/Pictures/cam_*.png')),
                          key=os.path.getmtime)
        if not pics:
            print("❌ No image found. Run camera_roi.py first or pass path arg.")
            sys.exit(1)
        path = pics[-1]
        print(f"Using latest: {path}")

    prepared_image_path = prepare_image(path)

    ip = get_ip()
    print()
    print(f"🌐 Mở browser:")
    print(f"    http://{ip}:{PORT}     (cùng mạng với Pi)")
    print(f"    http://localhost:{PORT}  (qua SSH port-forward -L {PORT}:localhost:{PORT})")
    print()
    print(f"ROI sequence: {', '.join(ROI_NAMES)} ({len(ROI_NAMES)*4} corners total)")
    print("Ctrl+C to stop server.")
    print()
    app.run(host='0.0.0.0', port=PORT, debug=False)


if __name__ == '__main__':
    main()
