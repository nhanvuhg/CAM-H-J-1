#!/usr/bin/env python3
"""Cham ROI len anh camera -> xuat YAML.

Khac roi_picker_web.py cu: file cu hard-code PROD_W/PROD_H = 1280x720 va resize
moi anh ve do. Toa do sinh ra khong con khop voi bbox (640x480) tu 6/2026, do
la nguyen nhan ROI hien tai bi lech. File nay cham o DUNG kich thuoc anh goc.

Dung:
    python3 roi_pick.py --cam cam0     # 1 outer + 5 row
    python3 roi_pick.py --cam cam1     # 10 slot
Roi mo http://<ip-pi>:8011 tren may co chuot.

Moi ROI = 4 goc, bam theo thu tu vong quanh (khong bat cheo).
"""
import argparse
import os
import socket
import sys

import cv2
from flask import Flask, Response, jsonify, request

PROFILES = {
    # ten ROI -> khop voi key trong vision_roi.yaml ma vision_decision_node doc
    'cam0': ['outer'] + [f'row{i}' for i in range(1, 6)],
    'cam1': [f'slot{i}' for i in range(1, 11)],
}
SECTION = {'cam0': 'input_tray', 'cam1': 'output_tray'}

app = Flask(__name__)
STATE = {}

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>ROI Pick</title>
<style>
 body{font-family:sans-serif;background:#111;color:#eee;margin:12px}
 #wrap{display:flex;gap:16px;flex-wrap:wrap}
 canvas{border:1px solid #444;cursor:crosshair;image-rendering:pixelated}
 #side{min-width:260px}
 button{padding:6px 10px;margin:2px;background:#2a2a2a;color:#eee;border:1px solid #555;border-radius:4px;cursor:pointer}
 button:hover{background:#3a3a3a}
 .done{color:#4caf50} .cur{color:#ffd54f;font-weight:bold} .todo{color:#888}
 li{margin:2px 0}
 pre{background:#000;padding:8px;max-height:240px;overflow:auto;font-size:11px}
</style></head><body>
<h3>ROI Pick — __CAM__ — anh goc __W__x__H__ (dung he bbox YOLO)</h3>
<div id="wrap">
 <div><canvas id="cv" width="__W__" height="__H__"></canvas>
  <div><button onclick="undo()">Undo diem</button>
       <button onclick="resetRoi()">Xoa ROI dang cham</button>
       <button onclick="save()">Xuat YAML</button>
       <span id="msg"></span></div></div>
 <div id="side"><b>Bam 4 goc moi ROI (vong quanh)</b><ul id="list"></ul><pre id="out"></pre></div>
</div>
<script>
const NAMES=__NAMES__, W=__W__, H=__H__;
const img=new Image(); img.src='/image';
const cv_=document.getElementById('cv'), ctx=cv_.getContext('2d');
let rois={}, idx=0, pts=[];
img.onload=()=>draw();
function cur(){return NAMES[idx];}
function draw(){
 ctx.drawImage(img,0,0,W,H);
 for(const [n,p] of Object.entries(rois)) poly(p,'#4caf50',n);
 poly(pts,'#ffd54f',cur());
 for(const p of pts){ctx.fillStyle='#ffd54f';ctx.beginPath();ctx.arc(p[0],p[1],3,0,7);ctx.fill();}
 let h='';
 NAMES.forEach((n,i)=>{const c=rois[n]?'done':(i==idx?'cur':'todo');
   h+=`<li class="${c}">${rois[n]?'✔':(i==idx?'▶':'·')} ${n}</li>`;});
 document.getElementById('list').innerHTML=h;
}
function poly(p,col,label){
 if(!p.length)return;
 ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(p[0][0],p[0][1]);
 for(let i=1;i<p.length;i++)ctx.lineTo(p[i][0],p[i][1]);
 if(p.length==4)ctx.closePath();
 ctx.stroke();
 if(p.length==4&&label){ctx.fillStyle=col;ctx.font='12px sans-serif';ctx.fillText(label,p[0][0]+3,p[0][1]-3);}
}
cv_.onclick=e=>{
 const r=cv_.getBoundingClientRect();
 // canvas hien thi co the bi CSS scale -> quy doi ve pixel anh goc
 const x=Math.round((e.clientX-r.left)*W/r.width), y=Math.round((e.clientY-r.top)*H/r.height);
 if(idx>=NAMES.length){document.getElementById('msg').textContent=' da du ROI';return;}
 pts.push([x,y]);
 if(pts.length==4){rois[cur()]=pts;pts=[];idx++;}
 draw();
};
function undo(){if(pts.length)pts.pop();else if(idx>0){idx--;pts=rois[cur()]||[];delete rois[cur()];pts.pop();}draw();}
function resetRoi(){pts=[];draw();}
function save(){
 fetch('/submit',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify({rois:rois})}).then(r=>r.json()).then(d=>{
   document.getElementById('out').textContent=d.yaml;
   document.getElementById('msg').textContent=' '+d.msg;});
}
</script></body></html>"""


@app.route('/')
def index():
    html = (PAGE.replace('__CAM__', STATE['cam'])
                .replace('__W__', str(STATE['w']))
                .replace('__H__', str(STATE['h']))
                .replace('__NAMES__', repr(PROFILES[STATE['cam']])))
    return Response(html, mimetype='text/html')


@app.route('/image')
def image():
    ok, buf = cv2.imencode('.png', STATE['img'])
    return Response(buf.tobytes(), mimetype='image/png')


@app.route('/submit', methods=['POST'])
def submit():
    rois = request.json.get('rois', {})
    names = PROFILES[STATE['cam']]
    missing = [n for n in names if n not in rois]

    lines = [f'# {STATE["cam"]} — cham tu {STATE["path"]}',
             '# ref_width/ref_height = kich thuoc anh luc cham. vision_decision_node',
             '# scale ROI tu day sang image_width/image_height cua no.',
             f'ref_width: {STATE["w"]}',
             f'ref_height: {STATE["h"]}',
             f'{SECTION[STATE["cam"]]}:']
    for n in names:
        if n in rois:
            pts = ', '.join(f'[{x}, {y}]' for x, y in rois[n])
            lines.append(f'  {n}: [{pts}]')
        else:
            lines.append(f'  # {n}: CHUA CHAM')
    yaml_text = '\n'.join(lines)

    out = os.path.expanduser(f'~/Pictures/roi/{STATE["cam"]}_roi.yaml')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        f.write(yaml_text + '\n')

    msg = f'luu {out}'
    if missing:
        msg += f' — CON THIEU: {", ".join(missing)}'
    print(f'[submit] {msg}')
    return jsonify(yaml=yaml_text, msg=msg)


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
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', choices=list(PROFILES), required=True)
    ap.add_argument('--image', default=None)
    ap.add_argument('--port', type=int, default=8011)
    args = ap.parse_args()

    path = args.image or os.path.expanduser(f'~/Pictures/roi/{args.cam}.png')
    if not os.path.exists(path):
        print(f'[LOI] khong thay {path} — chay grab_roi_frames.py truoc.')
        return 1
    img = cv2.imread(path)
    if img is None:
        print(f'[LOI] khong doc duoc {path}')
        return 1

    h, w = img.shape[:2]
    STATE.update(cam=args.cam, img=img, w=w, h=h, path=path)
    n = len(PROFILES[args.cam])
    print(f'{path} — {w}x{h} (KHONG resize) — can cham {n} ROI: {", ".join(PROFILES[args.cam])}')
    print(f'Mo:  http://{get_ip()}:{args.port}')
    app.run(host='0.0.0.0', port=args.port, debug=False)
    return 0


if __name__ == '__main__':
    sys.exit(main())
