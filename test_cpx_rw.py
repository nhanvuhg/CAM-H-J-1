import sys
import time
from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp

def main():
    ip = "172.16.11.37"
    print(f"Connecting to CPX at {ip}...")
    try:
        cpx = CpxAp(ip_address=ip, cycle_time=None)
        print("Connected!")
        mod = cpx.modules[3]
        
        # Read current state
        print("Initial output channels:")
        initial = mod.read_output_channels()
        print(f"  Ch0-9: {initial[:10]}")
        
        # Set channel 4 to True (Cylinder 1 retract)
        print("Setting channel 4 to True...")
        mod.set_channel(4)
        
        # Read back
        print("Output channels after setting Ch4:")
        after_set = mod.read_output_channels()
        print(f"  Ch0-9: {after_set[:10]}")
        
        # Reset channel 0
        print("Resetting channel 0 (using reset_channel)...")
        mod.reset_channel(0)
        
        # Read back
        print("Output channels after resetting Ch0:")
        after_reset = mod.read_output_channels()
        print(f"  Ch0-9: {after_reset[:10]}")
        
        # Cleanup
        cpx.shutdown()
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
