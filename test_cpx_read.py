import sys
from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp

def main():
    ip = "172.16.11.37"
    print(f"Connecting to CPX at {ip}...")
    try:
        cpx = CpxAp(ip_address=ip, cycle_time=None)
        print("Connected!")
        print(f"Modules: {len(cpx.modules)}")
        for idx, mod in enumerate(cpx.modules):
            print(f"Module {idx}: {mod.name} (Type: {type(mod).__name__})")
            if mod.is_function_supported("read_channels"):
                print(f"  Channels: {mod.read_channels()}")
            if mod.is_function_supported("read_output_channels"):
                print(f"  Output channels: {mod.read_output_channels()}")
        
        cpx.shutdown()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
