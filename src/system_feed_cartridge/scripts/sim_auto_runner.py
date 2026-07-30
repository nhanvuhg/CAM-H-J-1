#!/usr/bin/env python3
"""
Sim Auto Test — STATE1 Full Cycle (reactive).

Usage:
    source ~/ros2_ws/install/setup.bash
    python3 sim_auto_test.py

Flow:
  goto STATE1 → confirm_safe → [S1] → InX move → [S3] → InY jog → [S4]
  → row → extend → [S11] → InY→T1 → InY→T2 → retract → [S10]
  → InY→safe → InX→safe → COMPLETE
"""

import time, subprocess, sys, json

def pub(topic, mtype, data):
    try: subprocess.run(['ros2','topic','pub','--once',topic,mtype,data], capture_output=True, timeout=5)
    except: pass

def state(t=3):
    try:
        r = subprocess.run(['ros2','topic','echo','/system_state','--once'], capture_output=True, text=True, timeout=t)
        for l in r.stdout.splitlines():
            if l.startswith('data:'): return l.split(':',1)[1].strip()
    except: pass
    return None

def positions():
    try:
        r = subprocess.run(['ros2','topic','echo','/providesystem/servo_positions','--once'], capture_output=True, text=True, timeout=3)
        for l in r.stdout.splitlines():
            if l.startswith('data:'):
                raw = l.split(':',1)[1].strip().strip("'\"")
                if raw and raw != '{}': return json.loads(raw)
    except: pass
    return {}

def sim(sid, val=1):
    pub('/providesystem/sim_sensor', 'std_msgs/String', f"data: '{sid}:{val}'")
    print(f"      📡 S{sid} {'ON' if val else 'OFF'}")

def alive():
    r = subprocess.run(['pgrep','-c','-f','cartridge_providesystem_py'], capture_output=True, text=True)
    return r.stdout.strip() not in ('0','')

# ════════════════════════════════════════════════════
# State → Sensor mapping
# ════════════════════════════════════════════════════
# Mỗi state chờ sensor nào để tiếp tục
SENSOR_ACTIONS = {
    's1_inx_to_conveyor_end': {
        'sensor': 1, 'desc': 'S1 ON — có khay ở đầu conveyor',
    },
    's1_inx_wait_stop': {
        'sensor': None, 'desc': 'InX đang dừng... (auto)',
        'wait': 2,
    },
    's1_iny_search_tray': {
        'sensor': 4, 'desc': 'S4 ON — tray detected',
        'pre_sensor': 3,  # S3 trước (InX stopped confirm)
        'pre_desc': 'S3 ON — InX stopped, bắt đầu search',
        'jog_delay': 3,   # Chờ InY jog 3s trước khi S4
    },
    's1_cylinder1_extend': {
        'sensor': 11, 'desc': 'S11 ON — cylinder extended (gắp khay)',
    },
    's1_cylinder1_retract': {
        'sensor': 10, 'desc': 'S10 ON — cylinder retracted (nhả khay)',
    },
}

# States pass-through (servo move, auto-complete)
PASS_STATES = [
    's1_iny_confirm_safe',
    's1_inx_to_conveyor',
    's1_iny_to_nearest',
    's1_iny_to_target1',
    's1_iny_to_target2',
    's1_iny_return_safe',
    's1_inx_return_safe',
]


def main():
    print("╔═══════════════════════════════════════════════════════╗")
    print("║  🚀 SIM AUTO TEST — STATE1 Full Cycle                ║")
    print("╚═══════════════════════════════════════════════════════╝")

    if not alive():
        print("❌ Node chưa chạy!"); sys.exit(1)

    # ─── Clear sensors ───
    print("\n[0] Clear sim sensors...")
    for sid in [1,3,4,10,11,12,13]:
        sim(sid, 0)
    time.sleep(1)

    # ─── MANUAL + START + HOME ───
    print("\n[1] MANUAL + START → Homing...")
    pub('/providesystem/set_operation_mode','std_msgs/String','data: manual')
    time.sleep(2)
    pub('/system/start_button','std_msgs/Bool','data: true')
    
    t0 = time.time()
    while time.time()-t0 < 120:
        s = state()
        if s and 'idle' in s:
            print(f"  ✅ HOMED! ({int(time.time()-t0)}s)")
            break
        time.sleep(5)
    else:
        print("  ❌ Homing fail"); sys.exit(1)

    # ─── goto STATE1 ───
    print("\n[2] goto STATE1")
    pub('/providesystem/goto_state','std_msgs/String','data: STATE1')
    time.sleep(5)  # Cho state machine chạy qua các pass-through states
    st = state()
    print(f"  State sau goto: {st}")
    if st and 'idle' in st:
        # goto chưa apply, thử lại
        pub('/providesystem/goto_state','std_msgs/String','data: STATE1')
        time.sleep(5)
        st = state()
        print(f"  Retry → State: {st}")

    # ─── Reactive loop ───
    print("\n[3] Running STATE1 cycle...\n")
    
    last_st = None
    cycle = 1
    handled_in_cycle = set()  # Track which states handled per cycle
    
    for i in range(80):  # Max ~3 min
        time.sleep(1.5)
        st = state()
        if not st: continue

        if not alive():
            print(f"  ❌ NODE CRASHED at {st}!"); sys.exit(1)

        # Print state change
        if st != last_st:
            pos = positions()
            inx = pos.get('1', '?')
            iny = pos.get('2', '?')
            print(f"  [{i:2d}] {st:<35} InX={inx}  InY={iny}")
            
            # Detect new cycle (back to confirm_safe or conveyor from return)
            if last_st and ('return' in last_st or 'complete' in last_st):
                if 'confirm_safe' in st or 'conveyor' in st:
                    cycle += 1
                    handled_in_cycle.clear()
                    print(f"\n  ══ Cycle {cycle} ══")
            
            last_st = st

        # ── Handle state ──
        action_key = None
        for key in SENSOR_ACTIONS:
            if key in st:
                action_key = key
                break
        
        if action_key and action_key not in handled_in_cycle:
            handled_in_cycle.add(action_key)
            action = SENSOR_ACTIONS[action_key]
            
            # Wait if needed
            if action.get('wait'):
                time.sleep(action['wait'])
            
            # Pre-sensor (e.g., S3 before S4)
            if action.get('pre_sensor'):
                print(f"      → {action['pre_desc']}")
                sim(action['pre_sensor'])
                time.sleep(2)
            
            # Jog delay
            if action.get('jog_delay'):
                print(f"      ⏳ InY jogging {action['jog_delay']}s...")
                time.sleep(action['jog_delay'])
                pos = positions()
                print(f"      InY: {pos.get('2','?')}mm")
            
            # Main sensor
            if action.get('sensor'):
                print(f"      → {action['desc']}")
                sim(action['sensor'])
                time.sleep(2)
        
        # Pass-through states: just wait
        is_pass = False
        for ps in PASS_STATES:
            if ps in st:
                is_pass = True
                break
        
        # Done?
        if 'state1_complete' in st:
            print(f"\n  🎉 STATE1 COMPLETE!")
            break
        
        if 'idle' in st and i > 10:
            print(f"\n  ✅ Returned to idle — done.")
            break
        
        if 'error' in st and i > 3:
            print(f"\n  ⚠️ ERROR — resetting...")
            pub('/providesystem/goto_state','std_msgs/String','data: STATE1')
            handled_in_cycle.clear()
            time.sleep(3)

    # ═══ RESULT ═══
    print()
    st = state(); pos = positions(); ok = alive()
    print("╔═══════════════════════════════════════════════════════╗")
    print(f"║  State:  {(st or '?'):<44}║")
    print(f"║  InX:    {str(pos.get('1','?'))+'mm':<44}║")
    print(f"║  InY:    {str(pos.get('2','?'))+'mm':<44}║")
    print(f"║  Node:   {'✅ ALIVE' if ok else '❌ DEAD':<44}║")
    passed = ok and st and ('complete' in st or 'idle' in st)
    r = '🎉 PASSED' if passed else '⚠️ CHECK'
    print(f"║  Result: {r:<44}║")
    print("╚═══════════════════════════════════════════════════════╝")
    sys.exit(0 if passed else 1)

if __name__ == '__main__':
    main()
