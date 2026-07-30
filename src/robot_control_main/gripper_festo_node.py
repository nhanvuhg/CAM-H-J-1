#!/usr/bin/env python3
"""
Festo Gripper Controller Node
- Listens to /robot/gripper_cmd topic from C++ robot_logic_node
- Controls Festo CPX-AP gripper valve via CPX IO module
- No 'rich' dependency required

Channel map (CPX-AP module index 3 @ 172.16.11.37):
  ch0/ch1 = Gripper (open/close)
  ch2/ch3 = Picker  (open/close)
  ch8/ch9 = Cyl_loadcell    (ch8 = NHẢ/release coil, ch9 = KẸP/clamp coil)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

# Suppress rich dependency warning
import warnings
warnings.filterwarnings('ignore', message='.*rich.*')

try:
    from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp
    CPX_AVAILABLE = True
except ImportError as e:
    CPX_AVAILABLE = False
    print(f"⚠️  Warning: festo-cpx-io not available: {e}")
    print("   Running in simulation mode (commands will be logged only)")


class FestoGripperNode(Node):
    def __init__(self):
        super().__init__('festo_gripper_controller')
        
        # Get CPX IP from parameters (default: Festo gripper IP)
        self.declare_parameter('cpx_ip', '172.16.11.37')
        self.cpx_ip = self.get_parameter('cpx_ip').value
        
        # Get module index (default: module 3 for VABX valve terminal)
        self.declare_parameter('cpx_module_index', 3)
        self.cpx_module_index = self.get_parameter('cpx_module_index').value
        
        # Simulation mode flag
        self.simulation_mode = not CPX_AVAILABLE
        
        self.get_logger().info(f'Connecting to Festo CPX at {self.cpx_ip}...')
        
        # Initialize CPX connection (degraded mode allowed)
        self.myCPX = None
        self.myIO = None
        
        if CPX_AVAILABLE:
            try:
                self.myCPX = CpxAp(ip_address=self.cpx_ip)
                # Validate module index
                if self.cpx_module_index < len(self.myCPX.modules):
                    self.myIO = self.myCPX.modules[self.cpx_module_index]
                    # single logical name for module
                    self.myIO.name = "festo_io_module"
                    self.get_logger().info(f'✔ Connected to Festo CPX module {self.cpx_module_index}')
                    
                    # Force default state (DigitalOutput 1 and 2 = false) to ensure safety on startup
                    try:
                        channels = self.myIO.read_channels()
                        # Gripper: Open = Ch0 False, Ch1 False (Zero Voltage / Exhausted)
                        if len(channels) > 1 and (channels[0] or channels[1]):
                            self.get_logger().info('Initializing Gripper to default OPEN state (0V)...')
                            self.myIO.reset_channel(1)
                            self.myIO.reset_channel(0)
                        
                        # Picker: Open = Ch2 False, Ch3 False (Zero Voltage / Exhausted)
                        if len(channels) > 3 and (channels[2] or channels[3]):
                            self.get_logger().info('Initializing Picker to default OPEN state (0V)...')
                            self.myIO.reset_channel(3)
                            self.myIO.reset_channel(2)

                        # Cyl_loadcell: mặc định KẸP khi node khởi động.
                        # ch8 = NHẢ/release coil, ch9 = KẸP/clamp coil.
                        if len(channels) > 9:
                            self.get_logger().info('Initializing Cyl_loadcell to default CLAMP state (ch9 set)...')
                            self.myIO.reset_channel(8)
                            self.myIO.set_channel(9)

                        time.sleep(0.1)
                    except Exception as e:
                        self.get_logger().warn(f'Could not initialize valves: {e}')
                        
                else:
                    self.get_logger().error(f'✗ CPX module index {self.cpx_module_index} out of range')
                    self.myIO = None
                    self.simulation_mode = True
            except Exception as e:
                self.get_logger().error(f'✗ Failed to connect to Festo CPX: {e}')
                self.get_logger().warn('⚠️  Running in SIMULATION MODE')
                self.simulation_mode = True
        else:
            self.get_logger().warn('⚠️  CPX library not available - Running in SIMULATION MODE')

        # State flags for gripper and picker (separate devices/channels)
        self.gripper_open = True
        self.picker_open = True
        # Cyl_loadcell state: True = NHẢ/release, False = KẸP/clamp.
        self.cyl_loadcell_status = True

        # Subscriptions from robot_logic_node
        self.gripper_sub = self.create_subscription(
            Bool,
            '/robot/gripper_cmd',
            self.gripper_callback,
            10
        )

        # Optional picker commands (separate channels)
        self.picker_sub = self.create_subscription(
            Bool,
            '/robot/picker_cmd',
            self.picker_callback,
            10
        )

        # Cyl_loadcell cylinder commands (channels 8/9 on same CPX module)
        self.cyl_loadcell_sub = self.create_subscription(
            Bool,
            '/robot/cyl_loadcell_cmd',
            self.cyl_loadcell_callback,
            10
        )

        # Status publishers for feedback
        self.gripper_pub = self.create_publisher(Bool, '/robot/gripper_status', 10)
        self.picker_pub = self.create_publisher(Bool, '/robot/picker_status', 10)
        self.cyl_loadcell_pub = self.create_publisher(Bool, '/robot/cyl_loadcell_status', 10)
        self.cyl_loadcell_pub.publish(Bool(data=True))

        mode_str = "SIMULATION" if self.simulation_mode else "LIVE"
        self.get_logger().info(f'[{mode_str}] Waiting for commands on /robot/gripper_cmd, /robot/picker_cmd, /robot/cyl_loadcell_cmd...')
    
    def gripper_callback(self, msg: Bool):
        """
        Callback when gripper command received
        msg.data = True  → Gripper ON (close/grip)
        msg.data = False → Gripper OFF (open/release)
        """
        try:
            if msg.data:
                # Gripper ON: Close (grip)
                self.gripper_close()
            else:
                # Gripper OFF: Open (release)
                self.gripper_open_cmd()
        except Exception as e:
            self.get_logger().error(f'Error controlling gripper: {e}')


    def gripper_close(self):    #gripper_close — coil ch0/1
        """Close gripper - equivalent to setting valve"""
        if self.gripper_open:
            self.get_logger().info('🔽 Gripper: CLOSING (setting valve)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Gripper closed (channels: 0=reset, 1=set)')
                self.gripper_open = False
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                # Gripper uses channel 0 = open, 1 = close
                self.myIO.reset_channel(0)
                self.myIO.set_channel(1)
                self.gripper_open = False
                time.sleep(0.05)
                # Publish feedback
                msg = Bool()
                msg.data = True # Gripper closed = ON
                self.gripper_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to close gripper: {e}')
    
    def gripper_open_cmd(self):
        """Open gripper - reset valve to exhaust air"""
        if not self.gripper_open:
            self.get_logger().info('🔼 Gripper: OPENING (resetting valves to 0V)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Gripper opened (channels: 0,1=reset)')
                self.gripper_open = True
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                # Reset both to ensure no air/voltage
                self.myIO.reset_channel(1)
                self.myIO.reset_channel(0)
                time.sleep(0.05)
                self.gripper_open = True
                # Publish feedback
                msg = Bool()
                msg.data = False # Gripper opened = OFF
                self.gripper_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to open gripper: {e}')

    # Picker methods — use channels 2/3
    def picker_callback(self, msg: Bool):
        """Callback for picker commands (separate from gripper)"""
        try:
            if msg.data:
                self.picker_close()
            else:
                self.picker_open_cmd()
        except Exception as e:
            self.get_logger().error(f'Error controlling picker: {e}')

    def picker_close(self):
        """Close picker - set channel 3"""
        if self.picker_open:
            self.get_logger().info('🔽 Picker: CLOSING (setting valve)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Picker closed (channels: 3=set)')
                self.picker_open = False
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                self.myIO.reset_channel(2)
                self.myIO.set_channel(3)
                self.picker_open = False
                time.sleep(0.05)
                msg = Bool()
                msg.data = True # Picker closed = ON
                self.picker_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to close picker: {e}')

    def picker_open_cmd(self):
        """Open picker - reset picker channels to exhaust air"""
        if not self.picker_open:
            self.get_logger().info('🔼 Picker: OPENING (resetting valves to 0V)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Picker opened (channels: 2,3=reset)')
                self.picker_open = True
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                self.myIO.reset_channel(3)
                self.myIO.reset_channel(2)
                time.sleep(0.05)
                self.picker_open = True
                msg = Bool()
                msg.data = False # Picker opened = OFF
                self.picker_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to open picker: {e}')

    # =========================================================================
    # Cyl_loadcell cylinder — double-solenoid valve on CPX channels 8/9 (same module)
    # (Đã đổi dây khí ngoài thực tế)
    #   msg.data True  → NHẢ / EXTEND / RELEASE (set ch9, reset ch8)
    #   msg.data False → KẸP / RETRACT / CLAMP   (set ch8, reset ch9)
    # =========================================================================
    def cyl_loadcell_callback(self, msg: Bool):
        """Callback for cyl_loadcell commands. True = NHẢ (release), False = KẸP (clamp)."""
        try:
            if msg.data:
                self.cyl_loadcell_release()
            else:
                self.cyl_loadcell_clamp()
        except Exception as e:
            self.get_logger().error(f'Error controlling cyl_loadcell: {e}')

    def cyl_loadcell_release(self):
        """Cyl_loadcell NHẢ — energize release coil (ch9), clamp coil (ch8) off."""
        if not self.cyl_loadcell_status:
            self.get_logger().info('🟢 Cyl_loadcell: NHẢ / EXTEND (set ch9, reset ch8)')

            if self.simulation_mode:
                self.get_logger().info('[SIM] Cyl_loadcell released (channels: 8=reset, 9=set)')
                self.cyl_loadcell_status = True
                return

            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                self.myIO.reset_channel(8)
                self.myIO.set_channel(9)
                self.cyl_loadcell_status = True
                time.sleep(0.05)
                msg = Bool()
                msg.data = True  # Cyl_loadcell released/extended = True
                self.cyl_loadcell_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to release cyl_loadcell: {e}')

    def cyl_loadcell_clamp(self):
        """Cyl_loadcell KẸP — energize clamp coil (ch8), release coil (ch9) off."""
        if self.cyl_loadcell_status:
            self.get_logger().info('🔴 Cyl_loadcell: KẸP / RETRACT (set ch8, reset ch9)')

            if self.simulation_mode:
                self.get_logger().info('[SIM] Cyl_loadcell clamped (channels: 9=reset, 8=set)')
                self.cyl_loadcell_status = False
                return

            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                self.myIO.reset_channel(9)
                self.myIO.set_channel(8)
                self.cyl_loadcell_status = False
                time.sleep(0.05)
                msg = Bool()
                msg.data = False  # Cyl_loadcell clamped/retracted = False
                self.cyl_loadcell_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Failed to clamp cyl_loadcell: {e}')

    def shutdown(self):
        """Cleanup on shutdown"""
        try:
            self.get_logger().info('Shutting down Festo gripper controller...')
            
            if self.simulation_mode:
                self.get_logger().info('✔ Simulation mode shutdown complete')
                return
            
            # Try to open both before shutdown if possible
            try:
                self.gripper_open_cmd()
            except Exception:
                pass
            try:
                self.picker_open_cmd()
            except Exception:
                pass
            # Cyl_loadcell: giữ trạng thái an toàn NHẢ khi shutdown.
            try:
                if self.myIO:
                    self.myIO.reset_channel(8)
                    self.myIO.set_channel(9)
            except Exception:
                pass
            if self.myCPX:
                try:
                    self.myCPX.close()
                except Exception:
                    pass
            self.get_logger().info('✔ Festo CPX connection closed')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FestoGripperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested')
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
