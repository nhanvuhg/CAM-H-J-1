#!/usr/bin/env python3
import time
import threading
import sys
import rclpy

# Add the script dir to path so we can import the node
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from cartridge_providesystem_py_node import CartridgeSystem, CartridgeConfig

class MockMotionHandler:
    """Mock a Festo drive that hangs on get_position for 10 seconds to simulate a network socket drop"""
    def current_position(self):
        print("\n\033[93m[MODBUS MOCK]\033[0m current_position() called by background thread! Simulating 10s timeout block...\n")
        time.sleep(10.0)
        print("\n\033[93m[MODBUS MOCK]\033[0m 10s timeout block finished.\n")
        return 123.45
        
    def stop_motion_task(self): pass
    def shutdown(self): pass
    def acknowledge_faults(self): pass

def main():
    rclpy.init()
    config = CartridgeConfig()
    
    system = CartridgeSystem(config)
    
    # Inject the hanging mock into Servo 1
    system.servos[1] = MockMotionHandler()
    
    # Override the control loop to add a tracking print so we can visually see the thread is alive
    original_loop = system._control_loop
    system.tick_counter = 0
    def verbose_control_loop():
        if system.tick_counter % 20 == 0:  # Print every ~1 second (loop runs at 50ms)
            print(f"\033[92m[ROS MAIN THREAD]\033[0m _control_loop is ticking normally! tick={system.tick_counter}")
        system.tick_counter += 1
        original_loop()
        
    system._control_loop = verbose_control_loop
    
    print("="*70)
    print(" BẮT ĐẦU TEST SỰ CỐ: MÔ PHỎNG MẠNG TREO 10s (NETWORK TIMEOUT TEST)")
    print("="*70)
    print("Nếu giải pháp Thread thành công:")
    print("  -> Chữ màu xanh [ROS MAIN THREAD] vẫn sẽ nhảy liên tục không ngừng.")
    print("  -> Trong khi chữ màu vàng [MODBUS MOCK] bị kẹt 10 giây ở background.")
    print("  -> Loop sẽ KHÔNG bị cảnh báo 'treo 30s' từ watchdog.")
    print("="*70)
    print("Nhấn Ctrl+C để dừng test sau khi xem xong.")
    print("")
    
    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        print("\nTest kết thúc bởi người dùng.")
    finally:
        system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
