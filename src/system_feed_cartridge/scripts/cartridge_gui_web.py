#!/usr/bin/env python3
"""
Cartridge System GUI — Web Dashboard v5
Modes:
  AUTO   — Sensor thực tế, tự động trigger workflow. JOG bị khóa.
  MANUAL — Sim sensor + chọn STATE 1-4 trực tiếp. JOG bị khóa.
  JOG    — Điều khiển tự do các servo. Workflow bị khóa.
"""

import http.server
import json
import secrets
import threading
import time
import os
import signal
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Bool

PORT = 8080

# Use an explicit deployment token when provided; otherwise create a fresh
# per-process token instead of exposing a predictable credential.
AUTH_TOKEN = os.environ.get("GUI_AUTH_TOKEN") or secrets.token_urlsafe(32)

CONFIG_RANGES = {
    "inx_home":             (0.0,   200.0),
    "inx_safe_zone":        (0.0,   200.0),
    "inx_target":           (10.0,  700.0),
    "iny_home":             (0.0,   200.0),
    "iny_safe_zone":        (5.0,   200.0),
    "iny_place":            (10.0,  700.0),
    "outx_home":            (0.0,   200.0),
    "outx_target":          (10.0,  700.0),
    "outy_home":            (0.0,   200.0),
    "outy_target":          (10.0,  700.0),
    "outy_safe_zone":       (5.0,   200.0),
}
ROW_POSITION_RANGE = (0.0, 960.0)


def validate_config(config_dict: dict) -> tuple:
    table_key = config_dict.get("table", "")
    positions = config_dict.get("positions", {})
    if table_key and positions:
        lo, hi = ROW_POSITION_RANGE
        for row_str, val in positions.items():
            try:
                v = float(val)
            except (TypeError, ValueError):
                return False, f"Row {row_str}: giá trị không hợp lệ '{val}'"
            if not (lo <= v <= hi):
                return False, f"Row {row_str}: {v}mm ngoài phạm vi [{lo}, {hi}]mm"
        return True, ""
    for key, val in config_dict.items():
        if key in CONFIG_RANGES:
            try:
                v = float(val)
            except (TypeError, ValueError):
                return False, f"'{key}': giá trị không hợp lệ '{val}'"
            lo, hi = CONFIG_RANGES[key]
            if not (lo <= v <= hi):
                return False, f"'{key}': {v} ngoài phạm vi [{lo}, {hi}]"
    return True, ""


class GuiRosNode(Node):
    def __init__(self):
        super().__init__('cartridge_gui')
        qos = QoSProfile(depth=10)

        self._pubs = {
            '/providesystem/jog_cmd':           self.create_publisher(String, '/providesystem/jog_cmd', qos),
            '/providesystem/move_to_pos':        self.create_publisher(String, '/providesystem/move_to_pos', qos),
            '/providesystem/update_config':      self.create_publisher(String, '/providesystem/update_config', qos),
            '/providesystem/get_config':         self.create_publisher(String, '/providesystem/get_config', qos),
            '/providesystem/set_operation_mode': self.create_publisher(String, '/providesystem/set_operation_mode', qos),
            '/providesystem/goto_state':         self.create_publisher(String, '/providesystem/goto_state', qos),
            '/providesystem/set_target_row':     self.create_publisher(String, '/providesystem/set_target_row', qos),
            '/providesystem/hmi_resume':         self.create_publisher(Bool,   '/providesystem/hmi_resume', qos),
            '/providesystem/reset_faults':       self.create_publisher(String, '/providesystem/reset_faults', qos),
            '/providesystem/valve_cmd':          self.create_publisher(String, '/providesystem/valve_cmd', qos),
            '/system/start_button':              self.create_publisher(Bool,   '/system/start_button', qos),
            '/system/stop_button':               self.create_publisher(Bool,   '/system/stop_button', qos),
            '/system/pause_button':              self.create_publisher(Bool,   '/system/pause_button', qos),
            '/system/confirm_button':            self.create_publisher(Bool,   '/system/confirm_button', qos),
        }

        self._config_data    = None
        self._system_state   = 'UNKNOWN'
        self._current_mode   = ''
        self._notifications  = []
        self._servo_positions  = {}
        self._servo_velocities = {}
        self._jog_velocity_ms  = 0.05

        self.create_subscription(String, '/providesystem/config_data',    self._on_config, qos)
        self.create_subscription(String, '/system_state',                  self._on_state,  qos)
        self.create_subscription(String, '/providesystem/gui_notify',      self._on_notify, qos)
        self.create_subscription(String, '/providesystem/servo_positions', self._on_pos,    qos)
        self.create_subscription(String, '/providesystem/current_mode',    self._on_mode,   qos)

    def _on_config(self, msg):  self._config_data = msg.data
    def _on_state(self, msg):   self._system_state = msg.data
    def _on_mode(self, msg):    self._current_mode = msg.data.strip().lower()
    def _on_pos(self, msg):
        try:
            data = json.loads(msg.data)
            self._servo_velocities = data.pop('_vel', {})
            jv = data.pop('_jog_vel', None)
            if jv is not None:
                self._jog_velocity_ms = jv
            self._servo_positions = data
        except: pass

    def _on_notify(self, msg):
        try:
            data = json.loads(msg.data)
            data['time'] = time.strftime('%H:%M:%S')
            self._notifications.append(data)
            if len(self._notifications) > 30:
                self._notifications = self._notifications[-30:]
        except: pass

    def get_notifications(self, since_idx=0):
        return self._notifications[since_idx:]

    def pub_string(self, topic, data):
        pub = self._pubs.get(topic)
        if pub:
            msg = String(); msg.data = data; pub.publish(msg)
            return {"ok": True}
        return {"ok": False, "err": f"Unknown topic: {topic}"}

    def pub_bool(self, topic, data):
        pub = self._pubs.get(topic)
        if pub:
            msg = Bool(); msg.data = data; pub.publish(msg)
            return {"ok": True}
        return {"ok": False, "err": f"Unknown topic: {topic}"}


_ros_node = None


def get_ros_node(): return _ros_node


def init_ros():
    global _ros_node
    rclpy.init()
    _ros_node = GuiRosNode()
    threading.Thread(target=lambda: rclpy.spin(_ros_node), daemon=True).start()
    time.sleep(0.3)


def fast_pub_string(topic, data):
    node = get_ros_node()
    return node.pub_string(topic, data) if node else {"ok": False, "err": "ROS2 not init"}


def fast_pub_bool(topic, data):
    node = get_ros_node()
    return node.pub_bool(topic, data) if node else {"ok": False, "err": "ROS2 not init"}


class GUIHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args): pass

    def _auth(self) -> bool:
        token = self.headers.get("X-Auth-Token", "")
        if secrets.compare_digest(token, AUTH_TOKEN): return True
        qs = parse_qs(urlparse(self.path).query)
        return secrets.compare_digest(qs.get("token", [""])[0], AUTH_TOKEN)

    def _sec_headers(self):
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy",
                         "default-src 'self' 'unsafe-inline' https://fonts.googleapis.com "
                         "https://fonts.gstatic.com; connect-src 'self'")

    def _unauth(self):
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self._sec_headers(); self.end_headers()
        self.wfile.write(b'{"ok":false,"err":"Unauthorized"}')

    def do_GET(self):
        if not self._auth(): self._unauth(); return
        p = urlparse(self.path).path
        if p in ('/', '/index.html'): self._html(HTML)
        elif p == '/api/status':      self._json(self._get_status())
        elif p == '/api/config':      self._json(self._get_config())
        else: self.send_error(404)

    def do_POST(self):
        if not self._auth(): self._unauth(); return
        cl = int(self.headers.get('Content-Length', 0))
        if cl > 65536:
            self.send_response(413); self.end_headers()
            self.wfile.write(b'{"ok":false,"err":"Too large"}'); return
        body = self.rfile.read(cl)
        try:
            d = json.loads(body.decode()) if body else {}
        except json.JSONDecodeError:
            self.send_response(400); self.end_headers()
            self.wfile.write(b'{"ok":false,"err":"Invalid JSON"}'); return
        p = urlparse(self.path).path
        if p == '/api/update_config':
            ok, err = validate_config(d.get('config', {}))
            if not ok:
                self._json({"ok": False, "err": f"Validation: {err}"}); return
        routes = {
            '/api/start':          lambda: fast_pub_bool('/system/start_button', True),
            '/api/stop':           lambda: fast_pub_bool('/system/stop_button', True),
            '/api/pause':          lambda: fast_pub_bool('/system/pause_button', True),
            '/api/confirm':        lambda: fast_pub_bool('/system/confirm_button', True),
            '/api/hmi_resume':     lambda: fast_pub_bool('/providesystem/hmi_resume', True),
            '/api/goto_state':     lambda: fast_pub_string('/providesystem/goto_state', d.get('state', 'IDLE')),
            '/api/jog':            lambda: fast_pub_string('/providesystem/jog_cmd', d.get('cmd', '1 stop')),
            '/api/set_mode':       lambda: fast_pub_string('/providesystem/set_operation_mode', d.get('mode', 'auto')),
            '/api/set_target_row': lambda: fast_pub_string('/providesystem/set_target_row', str(d.get('row', '1'))),
            '/api/move_servo':     lambda: fast_pub_string('/providesystem/move_to_pos', d.get('cmd', '')),
            '/api/update_config':  lambda: fast_pub_string('/providesystem/update_config', json.dumps(d.get('config', {}))),
            '/api/get_config':     lambda: fast_pub_string('/providesystem/get_config', 'request'),
            '/api/reset_faults':   lambda: fast_pub_string('/providesystem/reset_faults', 'reset'),
            '/api/valve_cmd':      lambda: fast_pub_string('/providesystem/valve_cmd', d.get('cmd', '')),
            '/api/notifications':  lambda: self._get_notifs(d),
            '/api/positions':      lambda: self._get_pos(),
        }
        h = routes.get(p)
        if h: self._json(h())
        else: self.send_error(404)

    def _html(self, c):
        self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache'); self._sec_headers(); self.end_headers()
        self.wfile.write(c.replace('__REVPI_A_HOST__', REVPI_A_HOST).encode())

    def _json(self, data):
        self.send_response(200); self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*'); self._sec_headers(); self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def _get_status(self):
        node = get_ros_node()
        if node:
            return {"state": node._system_state, "mode": node._current_mode}
        return {"state": "UNKNOWN", "mode": ""}

    def _get_config(self):
        node = get_ros_node()
        if node and node._config_data:
            try: return {"ok": True, "config": json.loads(node._config_data)}
            except: pass
        return {"ok": False, "config": {}}

    def _get_notifs(self, d):
        node = get_ros_node()
        if node:
            since = d.get('since', 0)
            return {"ok": True, "notifications": node.get_notifications(since),
                    "total": len(node._notifications)}
        return {"ok": True, "notifications": [], "total": 0}

    def _get_pos(self):
        node = get_ros_node()
        if node:
            return {"ok": True,
                    "positions":    node._servo_positions,
                    "velocities":   node._servo_velocities,
                    "jog_velocity": node._jog_velocity_ms,
                    "jog_velocity_max": 0.08}
        return {"ok": True, "positions": {}, "velocities": {}, "jog_velocity": 0.05, "jog_velocity_max": 0.08}


# ============================================================
# HTML
# ============================================================

# IP RevPi A — đọc từ env-var REVPI_A_HOST (set trong ros2_env.sh, single
# source of truth). Iframe loadcell web GUI dùng placeholder __REVPI_A_HOST__
# trong HTML, được thay tại _html() trước khi serve.
REVPI_A_HOST = os.environ.get("REVPI_A_HOST", "172.16.11.31")

HTML = r"""<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cartridge System v5</title>
<link href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
:root{--bg:#0c0c1d;--bg2:#081e29;--card:#051a1a;--border:#134357;--accent:#4f6cff;
  --green:#00e676;--red:#ff5252;--orange:#ffa726;--yellow:#ffd740;
  --cyan:#26c6da;--teal:#5cf4f1;--text:#e8e8f0;--dim:#8888aa;--purple:#bb86fc;}
*{margin:0;padding:0;box-sizing:border-box;}
body{font-family:'JetBrains Mono',monospace;background:var(--bg);color:var(--text);
  height:100vh;overflow:hidden;font-size:12px;}
::-webkit-scrollbar{width:5px;}
::-webkit-scrollbar-thumb{background:var(--accent);border-radius:3px;}

/* ── Header ── */
.hdr{background:#141428;border-bottom:1px solid var(--border);padding:0 12px;
  height:44px;display:flex;align-items:center;gap:8px;z-index:10;}
.hdr h1{font-size:17px;font-weight:700;letter-spacing:.5px;white-space:nowrap;}
.hdr h1 span{color:var(--accent);}
.hdr-fill{flex:1;}
.badge{height:26px;border-radius:8px;padding:0 12px;font-size:12px;font-weight:700;
  display:flex;align-items:center;gap:7px;border:1px solid;white-space:nowrap;}
.badge-state{background:var(--bg);border-color:var(--border);}
.badge-homed{background:#051a1a;border-color:var(--border);font-size:11px;}
.badge-homed.homing{background:#2a1a00;border-color:var(--orange);color:var(--orange);}
.badge-homed.homed{background:#0a2e22;border-color:var(--green);color:var(--green);}
.badge-homed.not-homed{color:var(--dim);}
.sdot{width:9px;height:9px;border-radius:50%;background:#555;animation:pulse 2s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.mpill{height:26px;border-radius:20px;padding:0 14px;font-size:12px;font-weight:700;
  text-transform:uppercase;letter-spacing:1px;border:2px solid;
  display:flex;align-items:center;gap:6px;white-space:nowrap;}
.mp-none{background:#2a1a00;border-color:var(--yellow);color:var(--yellow);
  animation:blink .7s infinite alternate;}
@keyframes blink{from{opacity:.5}to{opacity:1}}
.mp-auto  {background:#0a332e;border-color:var(--green);color:var(--green);animation:none;}
.mp-ai    {background:#2d1a3a;border-color:#b462ff;color:#b462ff;animation:none;}
.mp-manual{background:#051a1a;border-color:var(--teal); color:var(--teal); animation:none;}
.mp-jog   {background:#332e0a;border-color:var(--orange);color:var(--orange);animation:none;}
.hdr-fault{height:26px;padding:0 10px;font-size:11px;font-weight:700;border-radius:4px;
  background:#3a1a0a;border:1px solid var(--orange);color:var(--orange);
  cursor:pointer;font-family:inherit;display:flex;align-items:center;gap:5px;}
.hdr-fault:hover{background:#4a2a10;}

/* ── Notify bar ── */
.nbar{display:none;padding:5px 14px;font-size:11px;font-weight:500;
  align-items:center;gap:10px;border-bottom:1px solid;}
.nbar.show{display:flex;}
.nbar.error{background:#2a0808;border-color:var(--red);color:var(--red);}
.nbar.warn {background:#2a1e08;border-color:var(--orange);color:var(--orange);}
.nbar.info {background:#081e28;border-color:var(--cyan);color:var(--cyan);}
.nbar .ntit{font-weight:700;white-space:nowrap;}
.nbar .ndet{flex:1;color:var(--text);opacity:.85;}
.nbar .ntm{font-size:10px;color:var(--dim);white-space:nowrap;}
.nbar .nact{display:flex;gap:5px;flex-shrink:0;}

/* ── Tabs ── */
.tabs{display:flex;gap:2px;background:#141428;border-bottom:1px solid var(--border);
  padding:3px 14px 0;height:30px;}
.tab{padding:3px 16px;font-size:13px;font-weight:600;border-radius:5px 5px 0 0;
  cursor:pointer;border:1px solid transparent;border-bottom:none;color:var(--dim);}
.tab.active{color:var(--accent);background:#141428;border-color:var(--border);}
.page{display:none;height:calc(100vh - 74px);}
.page.active{display:block;}

/* ── Grid ── */
.grid{display:grid;grid-template-columns:215px 1fr 248px;
  grid-template-rows:3fr 2.2fr;
  grid-template-areas:"lc cc sc" "lc log sc";
  gap:4px;padding:5px;height:100%;overflow:hidden;}
.lc{grid-area:lc;display:flex;flex-direction:column;gap:4px;overflow:hidden;}
.cc{grid-area:cc;display:flex;flex-direction:column;gap:4px;overflow:hidden;}
.sc{grid-area:sc;overflow-y:auto;}
.la{grid-area:log;}

/* ── Card ── */
.card{background:var(--bg2);border:1px solid var(--border);border-radius:6px;padding:8px;}
.card:hover{border-color:var(--accent);}
.ct{font-size:10px;font-weight:700;color:var(--accent);text-transform:uppercase;
  letter-spacing:1.5px;margin-bottom:5px;}

/* ── Buttons ── */
.btn{padding:5px 10px;border:1px solid var(--border);border-radius:4px;background:var(--card);
  color:var(--text);cursor:pointer;font-family:inherit;font-size:12px;font-weight:600;
  transition:all .12s;text-transform:uppercase;letter-spacing:.3px;white-space:nowrap;}
.btn:hover{border-color:var(--accent);background:#0a1e2e;}
.btn:active{transform:scale(.95);}
.g {background:#0a2e22;border-color:var(--green); color:var(--green);}
.r {background:#3a1010;border-color:var(--red);   color:var(--red);}
.o {background:#3a2a08;border-color:var(--orange);color:var(--orange);}
.t {background:#051a1a;border-color:var(--teal);  color:var(--teal);}
.ac{background:var(--accent);border-color:var(--accent);color:#fff;}
.bl{background:#0a1a3a;border-color:var(--accent);color:var(--accent);}
.gr2{display:grid;grid-template-columns:1fr 1fr;gap:4px;margin-bottom:4px;}
.gr3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin-bottom:4px;}
.gr2 .btn,.gr3 .btn{width:100%;padding:8px 4px;}
/* disabled state */
.dim-section{opacity:.32;pointer-events:none;}

/* ── Mode dropdown ── */
.mdrop{position:relative;}
.mdrop-header{width:100%;padding:7px 10px;background:var(--card);border:1px solid var(--border);
  border-radius:5px;cursor:pointer;display:flex;align-items:center;gap:8px;
  font-family:inherit;font-size:12px;font-weight:700;transition:border-color .15s;}
.mdrop-header:hover{border-color:var(--accent);}
.mdrop-header .mh-dot{width:8px;height:8px;border-radius:50%;background:var(--dim);flex-shrink:0;}
.mdrop-header .mh-txt{flex:1;text-align:left;}
.mdrop-header .mh-arr{color:var(--dim);font-size:10px;flex-shrink:0;}
.mdrop-opts{display:none;position:absolute;left:0;right:0;top:calc(100% + 4px);
  background:#141428;border:1px solid var(--border);border-radius:6px;z-index:50;
  padding:4px;box-shadow:0 8px 24px rgba(0,0,0,.5);}
.mdrop-opts.open{display:block;}
.mopt{padding:7px 10px;border-radius:4px;cursor:pointer;display:flex;align-items:center;
  gap:8px;border:1px solid transparent;transition:all .1s;margin-bottom:3px;}
.mopt:last-child{margin-bottom:0;}
.mopt:hover{border-color:currentColor;opacity:.9;}
.mopt-dot{width:8px;height:8px;border-radius:50%;flex-shrink:0;}
.mopt-name{font-size:12px;font-weight:700;}
.mopt-desc{font-size:9px;color:var(--dim);margin-top:1px;}
.mopt-auto  {color:var(--green); background:#0d3d2e;border-color:var(--green);}
.mopt-ai    {color:#b462ff; background:#291834;border-color:#b462ff;}
.mopt-manual{color:var(--teal);  background:#051a1a;border-color:var(--teal);}
.mopt-jog   {color:var(--orange);background:#332e0a;border-color:var(--orange);}

/* ── State nav buttons ── */
.state-btn{width:100%;padding:6px 4px;border-radius:4px;border:1px solid #00ffff;
  background:#0a1a3a;color:var(--accent);font-family:inherit;font-size:11px;
  font-weight:700;cursor:pointer;text-align:center;line-height:1.4;transition:all .12s;}
.state-btn:hover{background:#1a2a4a;border-color:#5cf4f1;}
.state-btn:active{transform:scale(.97);}
.state-btn.active-state{background:#0d3d2e;border-color:var(--green);color:var(--green);}
.state-btn.lk{opacity:.32;cursor:not-allowed;}
.state-btn.lk:hover{background:#0a1a3a;border-color:#00ffff;}

/* ── Servo cards ── */
.vel{background:var(--bg);border:1px solid var(--border);border-radius:5px;
  padding:3px 5px;width:50px;color:var(--text);font-size:11px;font-family:inherit;text-align:center;}
.vel:focus{outline:none;border-color:var(--accent);}
#svlist{display:flex;gap:4px;flex:1;min-height:0;overflow:hidden;}
.svc{flex:1;background:var(--card);border:1px solid var(--border);border-radius:4px;
  padding:6px;display:flex;flex-direction:column;align-items:center;gap:5px;
  overflow:hidden;min-width:0;}
.svc:hover{border-color:var(--accent);}
.svn{font-size:13px;font-weight:700;color:var(--cyan);}
.svd{font-size:9px;color:var(--dim);}
.svp{font-size:20px;font-weight:700;color:var(--yellow);}

/* ── Valve control ── */
.vlv-row{display:flex;gap:4px;}
.vlv-grp{flex:1;text-align:center;}
.vlv-grp .vlv-title{font-size:10px;font-weight:700;color:var(--purple);margin-bottom:3px;letter-spacing:.5px;}
.vlv-grp .btn{width:100%;padding:4px 2px;font-size:10px;font-weight:700;margin-bottom:2px;}
.jr{display:flex;gap:3px;width:100%;}
.jr .btn{flex:1;font-size:17px;font-weight:700;padding:9px 2px;}
.sfw{width:100%;}.sfw .btn{width:100%;font-size:14px;padding:9px 4px;}
.pr{display:flex;gap:3px;width:100%;align-items:center;}
.pi{flex:1;background:var(--bg);border:1px solid var(--border);border-radius:5px;
  padding:5px 3px;color:var(--text);font-size:14px;font-family:inherit;text-align:center;
  min-width:0;height:32px;}
.pi:focus{outline:none;border-color:var(--accent);}

/* ── Sensor panel ── */
.sgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:4px;margin-top:4px;}
.sb{display:flex;flex-direction:column;align-items:center;justify-content:center;
  padding:5px 3px;border-radius:4px;border:1px solid var(--border);background:var(--card);
  cursor:pointer;min-height:30px;transition:all .12s;}
.sb:hover{border-color:var(--cyan);}
.sb.on{background:#0a2e22;border-color:var(--green);}
.sb.on .sn{color:var(--green);}
.sn{font-size:11px;font-weight:700;}
.slb{font-size:8px;color:var(--dim);}
.sdt{width:6px;height:6px;border-radius:50%;background:#2a2a2a;margin-top:2px;}
.sb.on .sdt{background:var(--green);box-shadow:0 0 5px var(--green);}
.sb.lk{opacity:.32;cursor:not-allowed;}
.sb.lk:hover{border-color:var(--border);}

/* ── Log ── */
.lh{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;}
.lb{background:#080816;border:1px solid var(--border);border-radius:4px;
  padding:6px 8px;height:calc(100% - 26px);overflow-y:auto;
  font-size:12px;font-family:monospace;line-height:1.5;}
.lb .ok{color:var(--green);}.lb .er{color:var(--red);}
.lb .in{color:var(--cyan);}.lb .wn{color:var(--orange);}

/* ── Toast ── */
.tc{position:fixed;bottom:12px;right:12px;z-index:999;}
.ts{background:var(--card);border:1px solid var(--green);border-radius:6px;
  padding:6px 12px;margin-top:4px;font-size:11px;animation:si .2s;max-width:270px;}
.ts.er{border-color:var(--red);}.ts.wn{border-color:var(--orange);}
@keyframes si{from{transform:translateX(70px);opacity:0}to{transform:none;opacity:1}}

/* ── Config tab ── */
.cgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));
  gap:12px;padding:12px;overflow-y:auto;height:100%;}
.ct2{width:100%;border-collapse:collapse;}
.ct2 th{font-size:10px;text-transform:uppercase;color:var(--dim);
  padding:4px 7px;border-bottom:1px solid var(--border);}
.ct2 td{padding:3px 7px;border-bottom:1px solid #0a1a2a;}
.ct2 input{background:var(--bg);border:1px solid var(--border);border-radius:3px;
  padding:2px 4px;width:85px;color:var(--yellow);font-size:11px;
  font-family:monospace;text-align:right;}
.ct2 input:focus{outline:none;border-color:var(--accent);}
.rl{font-size:11px;font-weight:600;color:var(--cyan);}

/* ── Robot tab ── */
.rgrid{display:grid;grid-template-columns:280px 1fr;gap:12px;padding:12px;height:100%;overflow:hidden;}
.rcol{display:flex;flex-direction:column;gap:8px;overflow-y:auto;}
.robot-state-row{display:flex;align-items:center;gap:8px;padding:6px 8px;
  background:var(--card);border:1px solid var(--border);border-radius:4px;margin-bottom:3px;}
.robot-label{font-size:10px;color:var(--dim);text-transform:uppercase;letter-spacing:.8px;width:80px;flex-shrink:0;}
.robot-value{font-size:12px;font-weight:700;}
</style>
</head>
<body>

<!-- ════════════════ HEADER ════════════════ -->
<div class="hdr">
  <h1><span>Cartridge</span> System</h1>
  <div class="hdr-fill"></div>
  <!-- State badge -->
  <div class="badge badge-state">
    <div class="sdot" id="sdot"></div>
    <span id="stTxt">UNKNOWN</span>
  </div>
  <!-- Homing badge -->
  <div class="badge badge-homed not-homed" id="homedBadge">○ NOT HOMED</div>
  <div class="hdr-fill"></div>
  <!-- Mode pill -->
  <div class="mpill mp-none" id="mpill">⚠ SELECT MODE</div>
  <!-- Reset Faults -->
  <button class="hdr-fault" onclick="resetFaults()">🔄 Faults</button>
</div>

<!-- Notify bar -->
<div class="nbar" id="nb">
  <span class="ntit" id="ntit"></span>
  <span class="ndet" id="ndet"></span>
  <span class="ntm"  id="ntm"></span>
  <div class="nact">
    <button class="btn o" style="padding:2px 7px;font-size:10px" onclick="resetFaults()">Reset</button>
    <button class="btn"   style="padding:2px 7px;font-size:10px" onclick="dnb()">✕</button>
  </div>
</div>

<!-- ════════════════ TABS ════════════════ -->
<div class="tabs">
  <div class="tab active" onclick="tab('ctrl',this)">Control Dashboard</div>
  <div class="tab"        onclick="tab('hp',  this)">Fill HP Control</div>
  <div class="tab"        onclick="tab('cfg', this)">Technical System</div>
  <div class="tab"        onclick="tab('robot',this)">Robot Control</div>
</div>

<!-- ════════════════ CONTROL PAGE ════════════════ -->
<div class="page active" id="page-ctrl">
<div class="grid">

  <!-- Left col -->
  <div class="lc">

    <!-- System Control — đặt trên cùng để không bị dropdown Mode che -->
    <div class="card" id="sysCtrlCard">
      <div class="ct">System Control</div>
      <div id="sysCtrlInner">
        <div class="gr3">
          <button class="btn g" onclick="sys('start')">START</button>
          <button class="btn r" onclick="sys('stop')">STOP</button>
          <button class="btn o" onclick="sys('pause')">PAUSE</button>
        </div>
        <div class="gr2">
          <button class="btn t" onclick="api('/api/confirm')">Confirm</button>
          <button class="btn g" onclick="api('/api/hmi_resume')">Resume</button>
        </div>
      </div>
    </div>

    <!-- Mode Selection (dropdown) — đặt sau System Control -->
    <div class="card">
      <div class="ct">Mode Selection</div>
      <div class="mdrop" id="mdropWrap">
        <button class="mdrop-header" id="mdropHdr" onclick="toggleMdrop()">
          <span class="mh-dot" id="mhDot"></span>
          <span class="mh-txt" id="mhTxt">Chọn chế độ...</span>
          <span class="mh-arr" id="mhArr">▼</span>
        </button>
        <div class="mdrop-opts" id="mdropOpts">
          <div class="mopt mopt-auto" onclick="setMode('auto')">
            <span class="mopt-dot" style="background:var(--green)"></span>
            <div><div class="mopt-name">AUTO</div><div class="mopt-desc">Camera / Robot tín hiệu · JOG khóa</div></div>
          </div>
          <div class="mopt mopt-ai" onclick="setMode('ai')">
            <span class="mopt-dot" style="background:#b462ff"></span>
            <div><div class="mopt-name">AI MODE</div><div class="mopt-desc">Tự động + YOLO Vision · JOG khóa</div></div>
          </div>
          <div class="mopt mopt-manual" onclick="setMode('manual')">
            <span class="mopt-dot" style="background:var(--teal)"></span>
            <div><div class="mopt-name">MANUAL</div><div class="mopt-desc">Sim sensor · Chọn STATE 1–4 · JOG khóa</div></div>
          </div>
          <div class="mopt mopt-jog" onclick="setMode('jog')">
            <span class="mopt-dot" style="background:var(--orange)"></span>
            <div><div class="mopt-name">JOG</div><div class="mopt-desc">Di chuyển servo tự do · Workflow khóa</div></div>
          </div>
        </div>
      </div>
    </div>

    <!-- State Navigation -->
    <div class="card" id="stateNavCard">
      <div class="ct">State Navigation</div>
      <div id="stateNavInner">
        <!-- Top row: always available -->
        <div class="gr2" style="margin-bottom:6px">
          <button class="btn" onclick="gotoState('HOMING')">🏠 HOMING</button>
          <button class="btn" id="abortJogBtn" onclick="onAbortJog()">⛔ ABORT→JOG</button>
        </div>
        <!-- 4 workflow states -->
        <div style="font-size:9px;text-transform:uppercase;letter-spacing:.8px;color:var(--dim);margin-bottom:4px" id="wfLabel">
          — Workflow —
        </div>
        <div class="gr2">
          <button class="state-btn" id="bS1" onclick="gotoWf('STATE1')">
            ▶ STATE 1<br><span style="font-size:9px;font-weight:400;text-transform:none">Nạp khay In</span>
          </button>
          <button class="state-btn" id="bS2" onclick="gotoWf('STATE2')">
            ▶ STATE 2<br><span style="font-size:9px;font-weight:400;text-transform:none">Thay khay In</span>
          </button>
        </div>
        <div class="gr2" style="margin-top:4px">
          <button class="state-btn" id="bS3" onclick="gotoWf('STATE3')" style="border-color:var(--green);color:var(--green)">
            ▶ STATE 3<br><span style="font-size:9px;font-weight:400;text-transform:none">Cấp khay Out</span>
          </button>
          <button class="state-btn" id="bS4" onclick="gotoWf('STATE4')" style="border-color:var(--green);color:var(--green)">
            ▶ STATE 4<br><span style="font-size:9px;font-weight:400;text-transform:none">Thay khay Out</span>
          </button>
        </div>
        <div style="margin-top:4px">
          <button class="btn r" onclick="gotoState('ERROR')" style="width:100%;padding:4px">⛔ FORCE ERROR</button>
        </div>
      </div>
    </div>

    <!-- Valve Control -->
    <div class="card" style="flex-shrink:0">
      <div class="ct">Valve Control</div>
      <div style="display:flex;flex-direction:column;gap:4px">
        <div class="vlv-title">CYL 1</div>
        <button class="btn g" onclick="valveCmd('cyl1_extend')" style="width:100%;padding:4px;font-size:10px;font-weight:700">▶ Extend</button>
        <button class="btn o" onclick="valveCmd('cyl1_retract')" style="width:100%;padding:4px;font-size:10px;font-weight:700">◀ Retract</button>
        <div class="vlv-title" style="margin-top:4px">CYL 2</div>
        <button class="btn g" onclick="valveCmd('cyl2_extend')" style="width:100%;padding:4px;font-size:10px;font-weight:700">▶ Extend</button>
        <button class="btn o" onclick="valveCmd('cyl2_retract')" style="width:100%;padding:4px;font-size:10px;font-weight:700">◀ Retract</button>
      </div>
    </div>
  </div>

  <!-- Center col -->
  <div class="cc">
    <div class="card" style="flex-shrink:0">
      <div class="ct">Target Row</div>
      <div style="display:flex;gap:3px;flex-wrap:wrap" id="trbar"></div>
    </div>
    <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:0;overflow:hidden">
      <div style="display:flex;align-items:center;gap:6px;margin-bottom:5px;flex-shrink:0">
        <span class="ct" style="margin-bottom:0">Servo Control</span>
        <span style="font-size:10px;color:var(--dim)">JOG Vel:</span>
        <button class="btn" id="jvBtn" title="Nhấn để đặt tốc độ JOG"
          style="font-size:11px;padding:2px 9px;min-width:62px;border-color:var(--accent)"
          onclick="openVelModal()"><span id="jvDisp">0.050</span> m/s</button>
        <span style="font-size:10px;color:var(--dim)">|</span>
        <span style="font-size:10px;color:var(--dim)">FAS:</span>
        <span id="jvFas" style="font-size:11px;color:var(--cyan);font-family:monospace">-- mm/s</span>
        <span id="jlk" style="display:none;font-size:10px;color:var(--orange)">🔒 JOG mode only</span>
      </div>
      <div id="svlist"></div>
    </div>
  </div>

  <!-- Log -->
  <div class="card la">
    <div class="lh">
      <div class="ct" style="margin-bottom:0">Log</div>
      <button class="btn" style="padding:3px 9px;font-size:11px"
        onclick="document.getElementById('lb').innerHTML=''">Clear</button>
    </div>
    <div class="lb" id="lb"></div>
  </div>

  <!-- Sensor col -->
  <div class="sc"><div class="card" style="height:100%;display:flex;flex-direction:column">
    <div class="ct">Sensor Signal Display</div>

    <!-- Mọi mode (auto/manual/ai) đều đọc sensor THẬT từ IO module. Sim sensor đã deprecated. -->
    <div style="padding:4px 7px;background:#0a1f0a;border:1px solid var(--green);
      border-radius:4px;font-size:10px;color:var(--green);font-weight:700;margin-bottom:4px">
      📡 REAL SENSORS — read-only
    </div>

    <div style="font-size:9px;color:var(--dim);text-transform:uppercase;letter-spacing:1px;margin-bottom:4px">Status</div>
    <div class="sgrid" id="sg"></div>

    <div style="font-size:8px;color:var(--dim);line-height:1.5;margin-top:6px">
      <b>S1-S3</b> Băng tải · <b>S4</b> Scan Stack P1 · <b>S5</b> Output Det<br>
      <b>S6</b> Check Tray P1 · <b>S7</b> Tray@Robot<br>
      <b>S9/S10</b> Cyl1 Ret/Ext · <b>S11/S12</b> ATV Run/Fault<br>
      <b>S17</b> Platform · <b>S18</b> Feed OK<br>
      <b>S19</b> Check Tray P2 · <b>S20</b> Scan Stack P2<br>
      <b>S21/S22</b> Cyl2 Ret/Ext
    </div>
  </div></div>

</div></div>

<!-- ════════════════ TECHNICAL SYSTEM PAGE ════════════════ -->
<div class="page" id="page-cfg"><div class="cgrid">
  <div class="card"><div class="ct">Input Stack — InY (mm)</div>
    <table class="ct2"><tr><th>Row</th><th>Pos (mm)</th><th></th></tr></table>
    <table class="ct2" id="tIS"></table>
    <div style="margin-top:7px;display:flex;gap:5px">
      <button class="btn g" style="padding:3px 9px;font-size:11px" onclick="saveCfg('iny_input_stack','tIS')">Save</button>
      <button class="btn"   style="padding:3px 9px;font-size:11px" onclick="undoCfg('tIS')">↶ Back</button>
    </div>
  </div>
  <div class="card"><div class="ct">Output Stack — InY (mm)</div>
    <table class="ct2"><tr><th>Row</th><th>Pos (mm)</th><th></th></tr></table>
    <table class="ct2" id="tOS"></table>
    <div style="margin-top:7px;display:flex;gap:5px">
      <button class="btn g" style="padding:3px 9px;font-size:11px" onclick="saveCfg('iny_output_stack','tOS')">Save</button>
      <button class="btn"   style="padding:3px 9px;font-size:11px" onclick="undoCfg('tOS')">↶ Back</button>
    </div>
  </div>
  <div class="card"><div class="ct">Output Table — OutY (mm)</div>
    <table class="ct2"><tr><th>Row</th><th>Pos (mm)</th><th></th></tr></table>
    <table class="ct2" id="tOT"></table>
    <div style="margin-top:7px;display:flex;gap:5px">
      <button class="btn g" style="padding:3px 9px;font-size:11px" onclick="saveCfg('outy_output_table','tOT')">Save</button>
      <button class="btn"   style="padding:3px 9px;font-size:11px" onclick="undoCfg('tOT')">↶ Back</button>
    </div>
  </div>
  <div class="card"><div class="ct">Key Positions (mm)</div>
    <table class="ct2" id="tKP"><tr><th>Parameter</th><th>Value</th><th>Desc</th></tr></table>
  </div>
</div></div>

<!-- ════════════════ ROBOT CONTROL PAGE ════════════════ -->
<div class="page" id="page-robot"><div class="rgrid">

  <!-- Left: status + controls -->
  <div class="rcol">
    <div class="card">
      <div class="ct">Robot Status</div>
      <div class="robot-state-row">
        <span class="robot-label">System</span>
        <span class="robot-value" id="rSysState" style="color:var(--cyan)">—</span>
      </div>
      <div class="robot-state-row">
        <span class="robot-label">Mode</span>
        <span class="robot-value" id="rMode" style="color:var(--yellow)">—</span>
      </div>
      <div class="robot-state-row">
        <span class="robot-label">Homing</span>
        <span class="robot-value" id="rHomed">—</span>
      </div>
    </div>

    <div class="card">
      <div class="ct">Emergency</div>
      <button class="btn r" style="width:100%;padding:10px;font-size:14px;margin-bottom:6px"
        onclick="sys('stop')">🛑 EMERGENCY STOP</button>
      <button class="btn g" style="width:100%;padding:8px;font-size:13px"
        onclick="api('/api/hmi_resume')">▶ Resume</button>
    </div>

    <div class="card">
      <div class="ct">State Control</div>
      <div class="gr2">
        <button class="btn" onclick="gotoState('HOMING')" style="padding:8px">🏠 Homing</button>
        <button class="btn" onclick="gotoState('IDLE')"   style="padding:8px">⏹ Idle</button>
      </div>
      <button class="btn r" style="width:100%;padding:6px;margin-top:4px"
        onclick="gotoState('ABORT_TO_JOG')">⛔ Abort → JOG</button>
    </div>

    <div class="card">
      <div class="ct">Reset</div>
      <button class="btn o" style="width:100%;padding:8px" onclick="resetFaults()">🔄 Reset Faults</button>
    </div>
  </div>

  <!-- Right: log -->
  <div class="card" style="display:flex;flex-direction:column;overflow:hidden">
    <div class="lh">
      <div class="ct" style="margin-bottom:0">Robot Event Log</div>
      <button class="btn" style="padding:3px 9px;font-size:11px"
        onclick="document.getElementById('rlb').innerHTML=''">Clear</button>
    </div>
    <div class="lb" id="rlb"></div>
  </div>

</div></div>

<!-- ════════════════ FILL HP CONTROL PAGE ════════════════ -->
<div class="page" id="page-hp">
  <iframe src="http://__REVPI_A_HOST__:8080/" style="width:100%; height:100%; border:none; background:var(--bg);"></iframe>
</div>

<div class="tc" id="tc"></div>

<script>
// ─── State ──────────────────────────────────────────────
const SS={};for(let i=1;i<=22;i++)SS[i]=false;
let mode='', sysState='';

const SERVOS=[
  {id:1,name:'InX',    d:'Trục X đầu vào'},
  {id:2,name:'InY',    d:'Trục Y đầu vào'},
  {id:3,name:'PutTray',d:'Đẩy khay'},
  {id:4,name:'OutX',   d:'Trục X đầu ra'},
  {id:5,name:'OutY',   d:'Trục Y đầu ra'}
];
const SLB={
  1:'Belt start',2:'Belt mid',3:'Belt end',4:'Scan Stack P1',5:'Output det.',
  6:'Check Tray P1',7:'Tray@Robot',8:'[reserved]',9:'Cyl1 Ret',
  10:'Cyl1 Ext',11:'ATV Run',12:'ATV Fault',13:'[reserved]',
  14:'[res]',15:'[res]',16:'[res]',17:'Platform',
  18:'Feed OK',19:'Check Tray P2',20:'Scan Stack P2',
  21:'Cyl2 Ret',22:'Cyl2 Ext'
};

// ─── Mode ───────────────────────────────────────────────
function setMode(m) {
  document.getElementById('mdropOpts').classList.remove('open');
  document.getElementById('mhArr').textContent='▼';
  fetch('/api/set_mode',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:m})})
  .then(r=>r.json()).then(j=>{
    if(j.ok){ mode=m; updateModeUI(); log('Mode → '+m.toUpperCase(),'ok'); }
    else     { toast('❌ '+(j.err||'Mode change failed'),'er'); log(j.err,'er'); }
  }).catch(()=>toast('❌ Connection error','er'));
}

function toggleMdrop() {
  if(mode && sysState && sysState!=='idle' && sysState!=='unknown' && sysState!=='offline') {
    toast('⚠ Cannot change mode while running','wn'); return;
  }
  const o=document.getElementById('mdropOpts');
  o.classList.toggle('open');
  document.getElementById('mhArr').textContent=o.classList.contains('open')?'▲':'▼';
}

// Close dropdown on outside click
document.addEventListener('click',e=>{
  if(!document.getElementById('mdropWrap').contains(e.target)){
    document.getElementById('mdropOpts').classList.remove('open');
    document.getElementById('mhArr').textContent='▼';
  }
});

function updateModeUI() {
  const dot=document.getElementById('mhDot'), txt=document.getElementById('mhTxt');
  const pill=document.getElementById('mpill');
  const jlk=document.getElementById('jlk');

  // Header pill
  pill.className='mpill';
  if(!mode||mode==='idle'||mode===''){
    pill.className+=' mp-none'; pill.textContent='⚠ SELECT MODE';
    dot.style.background='var(--dim)'; txt.textContent='Chọn chế độ...'; txt.style.color='var(--dim)';
  } else if(mode==='auto'){
    pill.className+=' mp-auto';   pill.textContent='● AUTO';
    dot.style.background='var(--green)'; txt.textContent='AUTO'; txt.style.color='var(--green)';
  } else if(mode==='ai'){
    pill.className+=' mp-ai';     pill.textContent='● AI MODE';
    dot.style.background='#b462ff'; txt.textContent='AI MODE'; txt.style.color='#b462ff';
  } else if(mode==='manual'){
    pill.className+=' mp-manual'; pill.textContent='● MANUAL';
    dot.style.background='var(--teal)';  txt.textContent='MANUAL'; txt.style.color='var(--teal)';
  } else if(mode==='jog'){
    pill.className+=' mp-jog';    pill.textContent='● MANUAL (JOG)';
    dot.style.background='var(--orange)'; txt.textContent='MANUAL (JOG)'; txt.style.color='var(--orange)';
  }

  const noMode = !mode || mode==='' || mode==='idle';

  // System Control + State Nav: dim when no mode
  document.getElementById('sysCtrlInner').className = noMode ? 'dim-section' : '';
  document.getElementById('stateNavInner').className = noMode ? 'dim-section' : '';

  // JOG lock indicator
  const jogAllowed = mode==='jog';
  jlk.style.display = (!noMode && !jogAllowed) ? '' : 'none';
  document.querySelectorAll('.jb').forEach(b=>b.classList.toggle('lk', !jogAllowed));

  // Sensor display là read-only — luôn hiển thị real sensor, không phụ thuộc mode

  // Workflow state buttons: chỉ MANUAL được nhấn (AUTO/AI tự trigger; JOG khóa)
  const wfManualOnly = mode !== 'manual';
  ['bS1','bS2','bS3','bS4'].forEach(id=>{
    document.getElementById(id).classList.toggle('lk', wfManualOnly);
  });
  const wfl=document.getElementById('wfLabel');
  if(noMode)              { wfl.style.color='var(--dim)';    wfl.textContent='— Workflow —'; }
  else if(mode==='jog')   { wfl.style.color='var(--orange)'; wfl.textContent='— Workflow (JOG — locked) —'; }
  else if(mode==='auto' || mode==='ai') { wfl.style.color='var(--dim)'; wfl.textContent='— Workflow (' + mode.toUpperCase() + ' tự trigger) —'; }
  else                    { wfl.style.color='var(--teal)';   wfl.textContent='— Workflow — chọn để chạy —'; }

  // Abort→JOG button: shows JOG MODE when already in JOG
  const abtn=document.getElementById('abortJogBtn');
  if(mode==='jog'){ abtn.textContent='✅ JOG MODE'; abtn.style.borderColor='var(--orange)'; abtn.style.color='var(--orange)'; }
  else            { abtn.textContent='⛔ ABORT→JOG'; abtn.style.borderColor=''; abtn.style.color=''; }
}

// ─── API ────────────────────────────────────────────────
async function api(ep,data={}) {
  try {
    const r=await fetch(ep,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
    const j=await r.json();
    const lbl=ep.split('/').pop().replace(/_/g,' ');
    if(j.ok) toast('✅ '+lbl);
    else toast('❌ '+(j.err||'Error'),'er');
    log(lbl+': '+(j.ok?'OK':j.err), j.ok?'ok':'er');
    return j;
  } catch(e){ toast('❌ Connection','er'); return {ok:false}; }
}

function sys(a)       { api('/api/'+a); }
function resetFaults() { api('/api/reset_faults'); }
function gotoState(s)  { api('/api/goto_state',{state:s}); log('goto '+s,'in'); }
function valveCmd(cmd) { api('/api/valve_cmd',{cmd:cmd}); log('Valve: '+cmd,'in'); }

function gotoWf(s) {
  if(mode==='jog')  { toast('🔒 JOG mode — workflow locked','wn'); return; }
  if(!mode||mode===''||mode==='idle') { toast('⚠ Chọn mode trước','wn'); return; }
  api('/api/goto_state',{state:s}); log('Workflow → '+s,'ok');
}

function onAbortJog() {
  if(mode==='jog'){ toast('✅ Already in JOG mode','ok'); return; }
  api('/api/goto_state',{state:'ABORT_TO_JOG'}); log('ABORT_TO_JOG','wn');
}

function setRow(r) { api('/api/set_target_row',{row:r}); }

// ─── Sensor sim ─────────────────────────────────────────
// Sensor display — read-only refresh based on real sensor topic state
function rfs(id) {
  const el=document.getElementById('s'+id);
  if(el) el.className='sb'+(SS[id]?' on':'');
}

// ─── JOG ────────────────────────────────────────────────
function jog(id,dir) {
  if(mode!=='jog'){ toast('🔒 JOG: switch to JOG mode first','wn'); return; }
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:id+' '+dir})});
}
function jStop(id) {
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:id+' stop'})});
}
function openVelModal() {
  const cur=parseFloat(document.getElementById('jvDisp').textContent)||0.05;
  document.getElementById('jvInput').value=cur.toFixed(3);
  const m=document.getElementById('velModal');
  m.style.display='flex';
  setTimeout(()=>document.getElementById('jvInput').focus(),50);
}
function closeVelModal() {
  document.getElementById('velModal').style.display='none';
}
function applyVelModal() {
  const v=parseFloat(document.getElementById('jvInput').value);
  if(isNaN(v)||v<0.001||v>0.08){
    toast('❌ Velocity: 0.001 ~ 0.08 m/s','er'); return;
  }
  const vs=v.toFixed(3);
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:'0 set_jog_vel '+vs})});
  document.getElementById('jvDisp').textContent=vs;
  closeVelModal();
  toast('✅ JOG velocity: '+vs+' m/s','ok');
}
function hSv(id) {
  if(mode!=='jog'){ toast('🔒 JOG mode required','wn'); return; }
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'home '+id})});
  log('Home S'+id,'in');
}
function clSv(id) {
  if(mode!=='jog'){ toast('🔒 JOG mode required','wn'); return; }
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'clear '+id})});
  log('Clear S'+id,'in');
}
function mvSv(id) {
  if(mode!=='jog'){ toast('🔒 JOG mode required','wn'); return; }
  const inp=document.getElementById('pi'+id);
  const raw=(inp?.value||'').trim();
  if(!raw){ toast('❌ Chưa nhập vị trí','er'); return; }
  const pos=parseFloat(raw);
  if(isNaN(pos)){ toast('❌ Vị trí không hợp lệ: '+raw,'er'); return; }
  if(pos<-1000||pos>1000){ toast('❌ Vượt giới hạn (-1000..1000mm)','er'); return; }
  const sname=(SERVOS.find(x=>x.id===id)?.name)||('S'+id);
  fetch('/api/jog',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({cmd:id+' move '+pos.toFixed(2)})});
  log('RUN S'+id+' '+sname+' → '+pos.toFixed(2)+' mm','in');
  toast('▶ Move '+sname+' → '+pos.toFixed(2)+' mm','ok');
}

// ─── Numpad ─────────────────────────────────────────────
let _npTargetId=null;
function openNumpad(id) {
  if(mode!=='jog'){ toast('🔒 JOG mode required','wn'); return; }
  _npTargetId=id;
  const sname=(SERVOS.find(x=>x.id===id)?.name)||('S'+id);
  const cur=document.getElementById('pi'+id)?.value||'';
  const live=document.getElementById('sp'+id)?.textContent||'-- mm';
  document.getElementById('npTitle').textContent='S'+id+' · '+sname;
  document.getElementById('npCur').textContent=live;
  document.getElementById('npDisp').textContent=cur||'0';
  document.getElementById('numpadModal').style.display='flex';
}
function closeNumpad() {
  document.getElementById('numpadModal').style.display='none';
  _npTargetId=null;
}
function npKey(k) {
  const el=document.getElementById('npDisp');
  let v=el.textContent;
  if(v==='0'&&k!=='.') v='';
  if(k==='.'&&v.includes('.')) return;
  if(v.length>=10) return;
  el.textContent=v+k;
}
function npBack() {
  const el=document.getElementById('npDisp');
  const v=el.textContent;
  el.textContent=(v.length<=1)?'0':v.slice(0,-1);
}
function npClear() { document.getElementById('npDisp').textContent='0'; }
function npSign() {
  const el=document.getElementById('npDisp');
  let v=el.textContent;
  if(v==='0') return;
  el.textContent=v.startsWith('-')?v.slice(1):'-'+v;
}
function npOk() {
  if(_npTargetId===null) return;
  const v=document.getElementById('npDisp').textContent;
  const n=parseFloat(v);
  if(isNaN(n)){ toast('❌ Giá trị không hợp lệ','er'); return; }
  const inp=document.getElementById('pi'+_npTargetId);
  if(inp) inp.value=n.toFixed(2);
  log('Numpad S'+_npTargetId+' set '+n.toFixed(2)+' mm','in');
  closeNumpad();
}
function npRunNow() {
  if(_npTargetId===null) return;
  const id=_npTargetId;
  npOk();
  if(document.getElementById('pi'+id)?.value) mvSv(id);
}

// ─── Config ─────────────────────────────────────────────
async function loadCfg() {
  await fetch('/api/get_config',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  await new Promise(r=>setTimeout(r,500));
  const j=await(await fetch('/api/config')).json();
  if(!j.ok||!j.config) return;
  const c=j.config;
  if(c.iny_input_stack)   buildTbl('tIS',c.iny_input_stack);
  if(c.iny_output_stack)  buildTbl('tOS',c.iny_output_stack);
  if(c.outy_output_table) buildTbl('tOT',c.outy_output_table);
  if(c.operation_mode && c.operation_mode !== mode) {
    mode=c.operation_mode; updateModeUI();
  }
  buildKP(c);
}
const undoStacks = {tIS:[], tOS:[], tOT:[]};
function buildTbl(id,data) {
  const tbl=document.getElementById(id); tbl.innerHTML='';
  if(undoStacks[id]) undoStacks[id].length=0;
  for(let r=10;r>=1;r--) {
    const v=data[r]||data[String(r)]||0;
    const tr=document.createElement('tr');
    tr.innerHTML=`<td class="rl">R${r}</td><td><input type="number" step="0.1" value="${v}" id="${id}_r${r}"></td><td style="font-size:9px;color:var(--dim)">${r===10?'Top':r===1?'Bot':''}</td>`;
    tbl.appendChild(tr);
    const inp=tr.querySelector('input');
    inp.dataset.prev=String(v);
    inp.addEventListener('change',()=>{
      if(inp.value===inp.dataset.prev) return;
      undoStacks[id]?.push({row:r,prev:inp.dataset.prev});
      inp.dataset.prev=inp.value;
    });
  }
}
function undoCfg(id) {
  const st=undoStacks[id];
  if(!st||!st.length){ toast('Không có thay đổi để hoàn tác','wn'); return; }
  const last=st.pop();
  const inp=document.getElementById(`${id}_r${last.row}`);
  if(inp){ inp.value=last.prev; inp.dataset.prev=last.prev; }
}
function buildKP(c) {
  const t=document.getElementById('tKP');
  t.innerHTML='<tr><th>Parameter</th><th>Value</th><th>Desc</th></tr>';
  [['InX Home',c.inx_home,'S1 home'],['InX Target',c.inx_target,'500mm'],
   ['InY Home',c.iny_home,'S2 home'],['InY Place',c.iny_place,'200mm'],
   ['InY Safe',c.iny_safe_zone,'50mm']
  ].forEach(([n,v,d])=>{
    if(v===undefined) return;
    const tr=document.createElement('tr');
    tr.innerHTML=`<td class="rl">${n}</td><td style="color:var(--yellow)">${v}</td><td style="font-size:9px;color:var(--dim)">${d}</td>`;
    t.appendChild(tr);
  });
}
async function saveCfg(tbl,id) {
  const pos={};
  for(let r=1;r<=10;r++) {
    const el=document.getElementById(id+'_r'+r);
    if(el) pos[r]=parseFloat(el.value)||0;
  }
  await api('/api/update_config',{config:{table:tbl,positions:pos}});
}

// ─── Build UI ───────────────────────────────────────────
function buildRows() {
  const bar=document.getElementById('trbar');
  for(let r=10;r>=1;r--) {
    const b=document.createElement('button');
    b.className='btn';b.style.cssText='padding:4px 7px;font-size:11px;min-width:32px';
    b.textContent='R'+r;b.onclick=()=>setRow(r);bar.appendChild(b);
  }
}
function buildSensors() {
  // Read-only display — không gán onclick. Trạng thái cập nhật qua rfs() từ
  // topic /providesystem/sensors_state.
  const g=document.getElementById('sg');
  for(let i=1;i<=22;i++) {
    const d=document.createElement('div');
    d.className='sb';d.id='s'+i;
    const lb=SLB[i]||'';
    d.innerHTML=`<span class="sn">S${i}</span>${lb?`<span class="slb">${lb}</span>`:''}<span class="sdt"></span>`;
    g.appendChild(d);
  }
}
function buildServos() {
  const list=document.getElementById('svlist');
  SERVOS.forEach(s=>{
    const d=document.createElement('div');
    d.className='svc';
    d.innerHTML=`
      <div style="text-align:center"><div class="svn">S${s.id}: ${s.name}</div><div class="svd">${s.d}</div></div>
      <div class="svp" id="sp${s.id}">-- mm</div>
      <div style="font-size:9px;color:var(--dim);margin-top:-3px">V: <span id="sv${s.id}" style="color:var(--cyan);font-family:monospace">--</span> mm/s</div>
      <div class="jr">
        <button class="btn jb" onmousedown="jog(${s.id},'-')" onmouseup="jStop(${s.id})" onmouseleave="jStop(${s.id})" ontouchstart="jog(${s.id},'-')" ontouchend="jStop(${s.id})">−</button>
        <button class="btn r" onclick="jStop(${s.id})" style="font-size:13px">STOP</button>
        <button class="btn jb" onmousedown="jog(${s.id},'+')" onmouseup="jStop(${s.id})" onmouseleave="jStop(${s.id})" ontouchstart="jog(${s.id},'+')" ontouchend="jStop(${s.id})">+</button>
      </div>
      <div class="sfw"><button class="btn g jb" onclick="hSv(${s.id})">HOMING</button></div>
      <div class="sfw"><button class="btn o jb" onclick="clSv(${s.id})">CLEAR</button></div>
      <div class="pr">
        <input type="text" class="pi" id="pi${s.id}" placeholder="0.0"
          readonly inputmode="none"
          onclick="openNumpad(${s.id})"
          style="cursor:pointer;caret-color:transparent">
        <span style="font-size:9px;color:var(--dim)">mm</span>
        <button class="btn ac jb" onclick="mvSv(${s.id})" style="padding:5px 12px;font-size:15px">RUN</button>
      </div>`;
    list.appendChild(d);
  });
}

// ─── Toast & Log ────────────────────────────────────────
function toast(msg,type='ok') {
  const c=document.getElementById('tc');
  const t=document.createElement('div');
  t.className='ts'+(type==='er'?' er':type==='wn'?' wn':'');
  t.textContent=msg;c.appendChild(t);
  setTimeout(()=>t.remove(),3000);
}
function log(msg,type='in',target='lb') {
  const box=document.getElementById(target);
  if(!box) return;
  const tm=new Date().toLocaleTimeString('vi-VN');
  box.innerHTML+=`<div class="${type}">[${tm}] ${msg}</div>`;
  box.scrollTop=box.scrollHeight;
  while(box.children.length>200) box.removeChild(box.firstChild);
}
function rlog(msg,type='in') { log(msg,type,'rlb'); }

// ─── Polling ────────────────────────────────────────────
async function pollSt() {
  try {
    const j=await(await fetch('/api/status')).json();
    const s=(j.state||'').toUpperCase();
    const m=(j.mode||'').toLowerCase();
    sysState=s.toLowerCase();

    // State badge
    document.getElementById('stTxt').textContent=s||'UNKNOWN';
    const dot=document.getElementById('sdot');
    if(!s||s==='UNKNOWN'||s==='OFFLINE'){dot.style.background='#555';dot.style.boxShadow='';}
    else if(s.includes('ERROR')){dot.style.background='var(--red)';dot.style.boxShadow='0 0 7px var(--red)';}
    else if(s==='IDLE'){dot.style.background='var(--orange)';dot.style.boxShadow='0 0 7px var(--orange)';}
    else{dot.style.background='var(--green)';dot.style.boxShadow='0 0 7px var(--green)';}

    // Homing badge
    const hb=document.getElementById('homedBadge');
    if(sysState.indexOf('homing')!==-1){
      hb.className='badge badge-homed homing'; hb.textContent='⟳ HOMING...';
    } else if(m && m!=='' && (sysState==='idle'||sysState.indexOf('s1')!==-1||sysState.indexOf('state')!==-1)){
      hb.className='badge badge-homed homed'; hb.textContent='✓ HOMED';
    } else {
      hb.className='badge badge-homed not-homed'; hb.textContent='○ NOT HOMED';
    }

    // Active state buttons
    ['S1','S2','S3','S4'].forEach(n=>{
      const el=document.getElementById('b'+n);
      if(el) el.classList.toggle('active-state',
        s.indexOf(n+'_')!==-1 || s.indexOf('STATE'+n.slice(1))!==-1);
    });

    // Sync mode from server if different
    if(m && m!=='' && m!=='idle' && m!==mode){
      mode=m; updateModeUI();
    }

    // Robot tab
    document.getElementById('rSysState').textContent=s||'—';
    document.getElementById('rMode').textContent=(m||'—').toUpperCase();
    const hbClone=hb.textContent;
    document.getElementById('rHomed').textContent=hbClone;
    document.getElementById('rHomed').style.color=
      hbClone.includes('HOMED')&&!hbClone.includes('NOT') ? 'var(--green)'
      : hbClone.includes('HOMING') ? 'var(--orange)' : 'var(--dim)';

  } catch(e) {
    document.getElementById('stTxt').textContent='OFFLINE';
    sysState='offline';
  }
}
async function pollPos() {
  try {
    const j=await(await fetch('/api/positions',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'})).json();
    if(!j.ok) return;
    // Positions
    if(j.positions) {
      for(let i=1;i<=5;i++) {
        const el=document.getElementById('sp'+i);
        if(el){ const v=j.positions[String(i)]; el.textContent=v!==undefined?Number(v).toFixed(1)+' mm':'-- mm'; }
      }
    }
    // Actual velocities per servo (read from FAS drive)
    if(j.velocities) {
      let maxV=0;
      for(let i=1;i<=5;i++) {
        const el=document.getElementById('sv'+i);
        if(el){ const v=j.velocities[String(i)]; const vv=v!==undefined?Math.abs(Number(v)):null;
          el.textContent=vv!==null?vv.toFixed(1):'--';
          if(vv!==null && vv>maxV) maxV=vv; }
      }
      // Header: show max active velocity
      const hdr=document.getElementById('jvFas');
      if(hdr) hdr.textContent=maxV>0.5?maxV.toFixed(1)+' mm/s':'idle';
    }
    // Sync JOG velocity display from node
    if(j.jog_velocity!==undefined) {
      const disp=document.getElementById('jvDisp');
      if(disp) disp.textContent=parseFloat(j.jog_velocity).toFixed(3);
    }
  }catch(e){}
}
let _ni=0,_nt=null;
async function pollN() {
  try {
    const j=await(await fetch('/api/notifications',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({since:_ni})})).json();
    if(j.ok&&j.notifications&&j.notifications.length>0) {
      _ni=j.total;
      const last=j.notifications[j.notifications.length-1];
      showN(last.level,last.title,last.detail,last.time);
      j.notifications.forEach(n=>{
        const t=n.level==='error'?'er':n.level==='warn'?'wn':'in';
        log(n.title+(n.detail?': '+n.detail:''), t);
        rlog(n.title+(n.detail?': '+n.detail:''), t);
      });
    }
  }catch(e){}
}
function showN(lvl,title,det,tm) {
  const bar=document.getElementById('nb');
  const lmap={error:'error',warn:'warn',info:'info'};
  bar.className='nbar show '+(lmap[lvl]||'info');
  document.getElementById('ntit').textContent=title;
  document.getElementById('ndet').textContent=det||'';
  document.getElementById('ntm').textContent=tm||'';
  clearTimeout(_nt);
  if(lvl==='info') _nt=setTimeout(dnb,5000);
}
function dnb() { document.getElementById('nb').className='nbar'; }

function tab(n,el) {
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
  document.getElementById('page-'+n).classList.add('active');
  el.classList.add('active');
  if(n==='cfg') loadCfg();
}

// ─── Init ───────────────────────────────────────────────
buildRows(); buildSensors(); buildServos();
updateModeUI();
pollSt(); pollPos();
setInterval(pollSt,2000);
setInterval(pollN,1500);
setInterval(pollPos,500);
log('GUI v5 ready — chọn mode để bắt đầu','ok');
</script>

<!-- ════ JOG Velocity Modal ════ -->
<div id="velModal" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,.65);
  z-index:900;align-items:center;justify-content:center"
  onclick="if(event.target===this)closeVelModal()">
  <div style="background:var(--card);border:1px solid var(--border);border-radius:8px;
    padding:20px 24px;min-width:270px;max-width:320px;box-shadow:0 8px 32px #000a">
    <div style="font-size:13px;font-weight:700;color:var(--text);margin-bottom:12px">
      Đặt tốc độ JOG
    </div>
    <div style="display:flex;align-items:center;gap:8px;margin-bottom:6px">
      <input type="number" id="jvInput" class="vel"
        style="width:80px;font-size:14px;padding:5px 8px;text-align:right"
        min="0.001" max="0.08" step="0.001" value="0.050"
        onkeydown="if(event.key==='Enter')applyVelModal()">
      <span style="color:var(--dim);font-size:12px">m/s</span>
    </div>
    <div style="font-size:10px;color:var(--orange);margin-bottom:4px">
      ⚠ Tốc độ tối đa: <b>0.08 m/s</b> (80 mm/s) — theo firmware FAS
    </div>
    <div style="font-size:9px;color:var(--dim);margin-bottom:14px;line-height:1.5">
      Chỉ áp dụng cho lệnh JOG.<br>
      Tốc độ State và Homing <b>không thay đổi</b> — cấu hình trong FAS.
    </div>
    <div style="display:flex;gap:8px">
      <button class="btn g" style="flex:1;padding:7px;font-size:12px"
        onclick="applyVelModal()">Áp dụng</button>
      <button class="btn"   style="flex:1;padding:7px;font-size:12px"
        onclick="closeVelModal()">Hủy</button>
    </div>
  </div>
</div>

<!-- ════ Numpad Modal (move servo to position) ════ -->
<style>
.npk{padding:14px 0;font-size:18px;font-weight:700;background:var(--card);
  border:1px solid var(--border);border-radius:6px;color:var(--text);
  cursor:pointer;font-family:inherit;user-select:none;transition:background .08s;}
.npk:hover{background:#0a2640;}
.npk:active{background:#134357;}
.npk.op{background:#1a1428;border-color:var(--purple);color:var(--purple);}
.npk.ok{background:#0a2e22;border-color:var(--green);color:var(--green);}
.npk.cn{background:#3a1a0a;border-color:var(--orange);color:var(--orange);}
.npk.go{background:#082030;border-color:var(--cyan);color:var(--cyan);}
</style>
<div id="numpadModal" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,.7);
  z-index:950;align-items:center;justify-content:center"
  onclick="if(event.target===this)closeNumpad()">
  <div style="background:var(--card);border:1px solid var(--border);border-radius:10px;
    padding:18px 20px;min-width:300px;box-shadow:0 8px 32px #000a">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:8px">
      <div style="font-size:13px;font-weight:700;color:var(--text)">
        Nhập vị trí · <span id="npTitle" style="color:var(--cyan)">--</span>
      </div>
      <div style="font-size:10px;color:var(--dim)">live: <span id="npCur" style="color:var(--green);font-family:monospace">--</span></div>
    </div>
    <div id="npDisp" style="background:#000a;border:1px solid var(--border);
      border-radius:6px;padding:12px 14px;margin-bottom:10px;text-align:right;
      font-size:26px;font-weight:700;font-family:'JetBrains Mono',monospace;
      color:var(--green);min-height:36px;letter-spacing:1px">0</div>
    <div style="font-size:9px;color:var(--dim);text-align:right;margin:-6px 4px 10px 0">mm</div>
    <div style="display:grid;grid-template-columns:repeat(4,60px);gap:6px">
      <button class="npk" onclick="npKey('7')">7</button>
      <button class="npk" onclick="npKey('8')">8</button>
      <button class="npk" onclick="npKey('9')">9</button>
      <button class="npk op" onclick="npBack()">←</button>
      <button class="npk" onclick="npKey('4')">4</button>
      <button class="npk" onclick="npKey('5')">5</button>
      <button class="npk" onclick="npKey('6')">6</button>
      <button class="npk op" onclick="npClear()">C</button>
      <button class="npk" onclick="npKey('1')">1</button>
      <button class="npk" onclick="npKey('2')">2</button>
      <button class="npk" onclick="npKey('3')">3</button>
      <button class="npk op" onclick="npSign()">±</button>
      <button class="npk" onclick="npKey('0')">0</button>
      <button class="npk" onclick="npKey('.')">.</button>
      <button class="npk cn" onclick="closeNumpad()">Hủy</button>
      <button class="npk ok" onclick="npOk()">OK</button>
    </div>
    <button class="npk go" style="width:100%;margin-top:8px;padding:12px 0;font-size:14px"
      onclick="npRunNow()">▶ OK + RUN ngay</button>
  </div>
</div>
</body>
</html>
"""


def main():
    import socket
    print("  Initializing ROS2...")
    init_ros()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
    except:
        local_ip = "localhost"

    http.server.HTTPServer.allow_reuse_address = True
    server = http.server.HTTPServer(('0.0.0.0', PORT), GUIHandler)

    def _sig(sn, f):
        try: server.shutdown()
        except: pass
        try: rclpy.shutdown()
        except: pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    print("=" * 55)
    print("  Cartridge System GUI v5 — AUTO / MANUAL / JOG")
    print("=" * 55)
    print(f"  Local  : http://localhost:{PORT}?token={AUTH_TOKEN}")
    print(f"  Network: http://{local_ip}:{PORT}?token={AUTH_TOKEN}")
    print(f"  Token  : {AUTH_TOKEN}")
    print("=" * 55)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        try: server.shutdown()
        except: pass
        try: rclpy.shutdown()
        except: pass


if __name__ == '__main__':
    main()
