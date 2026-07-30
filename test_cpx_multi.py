import sys
import time
from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp

def run1():
    ip = "172.16.11.37"
    print("Run 1: Connecting...")
    cpx = CpxAp(ip_address=ip, cycle_time=None)
    mod = cpx.modules[3]
    print("Run 1: Setting Ch4 to True...")
    mod.set_channel(4)
    print(f"Run 1: Ch0-9 state: {mod.read_output_channels()[:10]}")
    cpx.shutdown()
    print("Run 1: Disconnected.")

def run2():
    ip = "172.16.11.37"
    print("Run 2: Connecting...")
    cpx = CpxAp(ip_address=ip, cycle_time=None)
    mod = cpx.modules[3]
    print(f"Run 2: Ch0-9 state on connect: {mod.read_output_channels()[:10]}")
    cpx.shutdown()
    print("Run 2: Disconnected.")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "2":
        run2()
    else:
        run1()
