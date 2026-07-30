import sys
import time
from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp

def setup_initial_state():
    ip = "172.16.11.37"
    print("Connecting to CPX to setup test state...")
    cpx = CpxAp(ip_address=ip, cycle_time=None)
    mod = cpx.modules[3]
    
    # Let's set some outputs:
    # Gripper: closed (ch0=True, ch1=False)
    # Picker: closed (ch2=True, ch3=False)
    # Cylinder 1: extended (ch4=False, ch5=True)
    # Cylinder 2: extended (ch8=False, ch9=True)
    # Cylinder 3: extended (ch6=True, ch7=False)
    
    print("Setting initial test state...")
    mod.reset_channel(1)
    mod.set_channel(0)
    
    mod.reset_channel(3)
    mod.set_channel(2)
    
    mod.reset_channel(4)
    mod.set_channel(5)
    
    mod.reset_channel(8)
    mod.set_channel(9)
    
    mod.reset_channel(7)
    mod.set_channel(6)
    
    time.sleep(0.5)
    print("Current output channels:")
    print(mod.read_output_channels()[:10])
    cpx.shutdown()
    print("Setup complete and disconnected.")

def simulate_startup():
    ip = "172.16.11.37"
    print("\nSimulating node startup...")
    cpx = CpxAp(ip_address=ip, cycle_time=None)
    mod = cpx.modules[3]
    
    print("Current outputs on connection:")
    print(mod.read_output_channels()[:10])
    
    # In gripper_festo_node.py on startup:
    # channels = self.myIO.read_channels()
    # if len(channels) > 1 and (channels[0] or channels[1]):
    #     self.myIO.reset_channel(1)
    #     self.myIO.reset_channel(0)
    # if len(channels) > 3 and (channels[2] or channels[3]):
    #     self.myIO.reset_channel(3)
    #     self.myIO.reset_channel(2)
    
    print("Running gripper_festo_node.py valve init...")
    channels = mod.read_channels()
    if len(channels) > 1 and (channels[0] or channels[1]):
        mod.reset_channel(1)
        mod.reset_channel(0)
    if len(channels) > 3 and (channels[2] or channels[3]):
        mod.reset_channel(3)
        mod.reset_channel(2)
        
    print("Current outputs after gripper_festo_node init:")
    print(mod.read_output_channels()[:10])
    
    # robot_logic_node publishes false to gripper and picker cmd topics 2 seconds after startup.
    # When cartridge_providesystem_py_node receives gripper_cmd = false:
    # valve_mod.reset_channel(0)
    # valve_mod.set_channel(1)
    # When it receives picker_cmd = false:
    # valve_mod.reset_channel(2)
    # valve_mod.set_channel(3)
    
    print("Simulating robot_logic_node publishing false after 2s...")
    mod.reset_channel(0)
    mod.set_channel(1)
    mod.reset_channel(2)
    mod.set_channel(3)
    
    print("Current outputs after robot_logic_node false commands:")
    print(mod.read_output_channels()[:10])
    
    cpx.shutdown()
    print("Simulated startup complete.")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "run":
        simulate_startup()
    else:
        setup_initial_state()
