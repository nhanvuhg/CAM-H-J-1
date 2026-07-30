"""Check STO and digital IO status via PNUs"""
import time
from edcon.edrive.com_modbus import ComModbus

IP = "172.16.11.35"
com = ComModbus(ip_address=IP, cycle_time=60, timeout_ms=3000)

pnus = [
    (898, "STO status"),
    (899, "SBC status"),
    (840, "Digital input status"),
    (841, "Digital output status"),
    (834, "Fault code active"),
    (835, "Warning code active"),
    (947, "Drive ready"),
    (942, "Position actual"),
]

for pnu, desc in pnus:
    try:
        data = com.read_pnu_raw(pnu)
        if data:
            val = int.from_bytes(data[:4], 'little')
            print(f"PNU {pnu:4d} ({desc:25s}): {val} (hex: {data[:4].hex()}, bin: {bin(val)})")
        else:
            print(f"PNU {pnu:4d} ({desc:25s}): read failed")
    except Exception as e:
        print(f"PNU {pnu:4d} ({desc:25s}): error: {e}")

com.shutdown()
