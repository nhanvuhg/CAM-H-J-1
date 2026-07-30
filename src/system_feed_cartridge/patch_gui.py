import sys

with open('/home/pi/ros2_ws/src/system_feed_cartridge/scripts/cartridge_gui.py', 'r') as f:
    code = f.read()

# 1. CSS
code = code.replace(
    ".mp-auto  {background:#0a332e;border-color:var(--green);color:var(--green);animation:none;}",
    ".mp-auto  {background:#0a332e;border-color:var(--green);color:var(--green);animation:none;}\n.mp-ai    {background:#2d1a3a;border-color:#b462ff;color:#b462ff;animation:none;}"
)
code = code.replace(
    ".mopt-auto  {color:var(--green); background:#0d3d2e;border-color:var(--green);}",
    ".mopt-auto  {color:var(--green); background:#0d3d2e;border-color:var(--green);}\n.mopt-ai    {color:#b462ff; background:#291834;border-color:#b462ff;}"
)

# 2. HTML Mode Option
mopt_auto = """          <div class="mopt mopt-auto" onclick="setMode('auto')">
            <span class="mopt-dot" style="background:var(--green)"></span>
            <div><div class="mopt-name">AUTO</div><div class="mopt-desc">Camera / Robot tín hiệu · JOG khóa</div></div>
          </div>"""
mopt_ai = """          <div class="mopt mopt-ai" onclick="setMode('ai')">
            <span class="mopt-dot" style="background:#b462ff"></span>
            <div><div class="mopt-name">AI MODE</div><div class="mopt-desc">Tự động + YOLO Vision · JOG khóa</div></div>
          </div>"""

code = code.replace(mopt_auto, mopt_auto + "\n" + mopt_ai)

# 3. JS updateModeUI
js_auto = """  } else if(mode==='auto'){
    pill.className+=' mp-auto';   pill.textContent='● AUTO';
    dot.style.background='var(--green)'; txt.textContent='AUTO'; txt.style.color='var(--green)';"""
js_ai = """  } else if(mode==='ai'){
    pill.className+=' mp-ai';     pill.textContent='● AI MODE';
    dot.style.background='#b462ff'; txt.textContent='AI MODE'; txt.style.color='#b462ff';"""

code = code.replace(js_auto, js_auto + "\n" + js_ai)

# 4. mode==='auto' checks
code = code.replace(
    "document.getElementById('rsbadge').style.display = mode==='auto' ? '' : 'none';",
    "document.getElementById('rsbadge').style.display = (mode==='auto' || mode==='ai') ? '' : 'none';"
)
code = code.replace(
    "document.querySelectorAll('.sb').forEach(b=>b.classList.toggle('lk', mode==='auto'));",
    "document.querySelectorAll('.sb').forEach(b=>b.classList.toggle('lk', mode==='auto' || mode==='ai'));"
)
code = code.replace(
    "else if(mode==='auto')  { wfl.style.color='var(--dim)';    wfl.textContent='— Workflow (auto trigger) —'; }",
    "else if(mode==='auto' || mode==='ai')  { wfl.style.color='var(--dim)';    wfl.textContent='— Workflow (' + mode.toUpperCase() + ' trigger) —'; }"
)

# 5. Sensor Sim checks
code = code.replace("if(mode==='auto'){ toast('🔒 Sim: not available in AUTO','wn'); return; }", "if(mode==='auto' || mode==='ai'){ toast('🔒 Sim: not available in AUTO/AI','wn'); return; }")
code = code.replace("if(mode==='auto'){ toast('🔒 AUTO mode','wn'); return; }", "if(mode==='auto' || mode==='ai'){ toast('🔒 AUTO/AI mode','wn'); return; }")
code = code.replace("+(mode==='auto'?' lk':'');", "+((mode==='auto' || mode==='ai')?' lk':'');")

with open('/home/pi/ros2_ws/src/system_feed_cartridge/scripts/cartridge_gui.py', 'w') as f:
    f.write(code)

print("Patch applied")
