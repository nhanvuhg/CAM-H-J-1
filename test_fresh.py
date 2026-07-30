import time
from edcon.edrive.com_modbus import ComModbus
from edcon.edrive.motion_handler import MotionHandler

for ip, name in [("172.16.11.35", "S1"), ("172.16.11.36", "S2")]:
    print(f"\n=== {name} ({ip}) ===")
    try:
        com = ComModbus(ip_address=ip, cycle_time=60, timeout_ms=3000)
        mot = MotionHandler(com)
        time.sleep(1.0)
        mot.acknowledge_faults()
        time.sleep(0.5)
        mot.update_inputs()
        z = mot.telegram.zsw1
        print(f"ZSW1: ready={z.ready_to_switch_on} inhibited={z.switching_on_inhibited} op={z.operation_enabled} fault={z.fault_present} ctrl={z.control_requested}")
        print("enable_powerstage()...")
        result = mot.enable_powerstage()
        print(f"Result: {result}")
        mot.update_inputs()
        z2 = mot.telegram.zsw1
        print(f"After: ready={z2.ready_to_switch_on} inhibited={z2.switching_on_inhibited} op={z2.operation_enabled} ctrl={z2.control_requested}")
        if z2.operation_enabled:
            print("DRIVE READY! velocity_task(30)...")
            mot.velocity_task(30, duration=0.0)
            time.sleep(1)
            p1 = mot.current_position()
            time.sleep(1)
            p2 = mot.current_position()
            print(f"Moved: {p2-p1} counts")
            mot.stop_motion_task()
        com.shutdown()
    except Exception as e:
        print(f"Error: {e}")
