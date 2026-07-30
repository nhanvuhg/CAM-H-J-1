#!/usr/bin/env python3
"""
Test script: Verify Modbus connection + MOVE command
Moves both servos to position 1000000 counts (~10mm) then back to 0.
"""
import time
import sys

from edcon.utils.logging import Logging
from edcon.edrive.com_modbus import ComModbus
from edcon.edrive.motion_handler import MotionHandler

Logging()

def safe_try(func, retries=3, backoff=2):
    for attempt in range(1, retries + 1):
        try:
            return func()
        except ConnectionError as e:
            print(f"  ⚠️  ConnectionError attempt {attempt}/{retries}: {e}")
            if attempt == retries:
                raise
            time.sleep(backoff * attempt)

SERVO_IPS = {
    1: '172.16.11.35',
    2: '172.16.11.36',
}

MOVE_TARGET = 1000000   # 10mm in counts (100000 counts/mm)
MOVE_VELOCITY = 300
MOVE_TIMEOUT = 30.0

print("=" * 60)
print("🔧 Festo Servo MOVE Test")
print(f"   Target: {MOVE_TARGET} counts ({MOVE_TARGET/100000:.1f}mm)")
print(f"   Velocity: {MOVE_VELOCITY}")
print("=" * 60)

# Step 1: Connect
connected = {}
for servo_id, ip in SERVO_IPS.items():
    print(f"\n--- Connecting Servo {servo_id} ({ip}) ---")
    try:
        com = ComModbus(ip_address=ip, cycle_time=60, timeout_ms=15000)
        mot = MotionHandler(com)
        safe_try(lambda: mot.acknowledge_faults())
        safe_try(lambda: mot.enable_powerstage())
        if not mot.referenced():
            print(f"  Homing servo {servo_id}...")
            safe_try(lambda: mot.referencing_task())
        print(f"  ✅ Servo {servo_id} ready, referenced={mot.referenced()}")
        connected[servo_id] = mot
    except Exception as e:
        print(f"  ❌ Failed: {e}")

if not connected:
    print("❌ No servos connected, aborting")
    sys.exit(1)

# Step 2: Move to target (non-blocking)
print(f"\n{'=' * 60}")
print(f"📦 Moving to {MOVE_TARGET} counts ({MOVE_TARGET/100000:.1f}mm)...")
print(f"{'=' * 60}")

for servo_id, mot in connected.items():
    try:
        safe_try(lambda m=mot: m.position_task(MOVE_TARGET, MOVE_VELOCITY, nonblocking=True))
        print(f"  Servo {servo_id}: move command sent")
    except Exception as e:
        print(f"  Servo {servo_id}: move failed: {e}")

# Step 3: Poll until all reach target
start = time.time()
while time.time() - start < MOVE_TIMEOUT:
    all_reached = True
    status = []
    for servo_id, mot in connected.items():
        reached = mot.target_position_reached()
        status.append(f"S{servo_id}={'✅' if reached else '⏳'}")
        if not reached:
            all_reached = False
    
    elapsed = time.time() - start
    print(f"  [{elapsed:.1f}s] {' | '.join(status)}")
    
    if all_reached:
        print(f"\n🎉 ALL reached target in {elapsed:.1f}s!")
        break
    time.sleep(0.5)
else:
    print(f"\n⏰ TIMEOUT after {MOVE_TIMEOUT}s!")

# Step 4: Move back to 0
print(f"\n{'=' * 60}")
print(f"� Moving back to 0...")
print(f"{'=' * 60}")

for servo_id, mot in connected.items():
    try:
        safe_try(lambda m=mot: m.position_task(0, MOVE_VELOCITY, nonblocking=True))
        print(f"  Servo {servo_id}: return command sent")
    except Exception as e:
        print(f"  Servo {servo_id}: return failed: {e}")

start = time.time()
while time.time() - start < MOVE_TIMEOUT:
    all_reached = True
    status = []
    for servo_id, mot in connected.items():
        reached = mot.target_position_reached()
        status.append(f"S{servo_id}={'✅' if reached else '⏳'}")
        if not reached:
            all_reached = False
    
    elapsed = time.time() - start
    print(f"  [{elapsed:.1f}s] {' | '.join(status)}")
    
    if all_reached:
        print(f"\n🎉 ALL returned to 0 in {elapsed:.1f}s!")
        break
    time.sleep(0.5)
else:
    print(f"\n⏰ TIMEOUT returning!")

# Cleanup
print(f"\n{'=' * 60}")
for servo_id, mot in connected.items():
    try:
        mot.shutdown()
        print(f"  Servo {servo_id} shutdown OK")
    except Exception as e:
        print(f"  Servo {servo_id} shutdown error: {e}")

print("✅ Test complete!")
