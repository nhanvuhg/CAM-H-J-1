#!/usr/bin/env python3
"""Cong cu chan doan kenh analog cua RevPi AIO — chay TREN RevPi.

Khong dung ROS, khong dung chung tai nguyen voi loadcell_node (mo o che do
monitoring=True nen chi doc, khong ghi process image).

Vi sao can: piTest va revpimodio2 doc CUNG mot thanh ghi 16 bit nhung dien
giai dau khac nhau.

    piTest -r InputValue_4  ->  65075 dez (0xFE33)   = doc KHONG DAU
    revpimodio2 io.value    ->  -461                 = doc CO DAU

65075 - 65536 = -461, cung mot gia tri. Neu node lay nham ban khong dau thi
-0.46 mA se thanh 65.075 mA — khong roi vao nhanh FAULT, va quy ra 5000 g
(cham tran) thay vi bao mat tin hieu. Script nay in ca hai cach dien giai de
xac dinh revpimodio2 tra ve kieu nao tren may nay.

Dung:
    python3 aio_probe.py                 # theo doi kenh 4, module 32
    python3 aio_probe.py --pos 32 --ch 4
    python3 aio_probe.py --all           # quet moi kenh cua moi module AIO
    python3 aio_probe.py --seconds 30    # chay lau hon roi tong ket
"""

import argparse
import sys
import time
import warnings

warnings.filterwarnings('ignore')

try:
    import revpimodio2
except ImportError:
    sys.exit('khong import duoc revpimodio2 — script nay phai chay tren RevPi')

# Mac dinh phai KHOP voi tham so node dang chay, neu khong cot `gram` o day se
# lech han so tren GUI va gay chan doan sai. 17/08/2026: sau khi can chinh bang
# qua chuan 2300g, start_loadcell.sh tren RevPi dat max_current_mA:=17.546 (SPAN
# cua KM02A cho 2.7 uA/gam thay vi 3.2). Doi can chinh thi sua CA HAI cho, hoac
# truyen --max-ma khi chay.
MIN_MA, MAX_MA, MAX_CAP_G = 4.0, 17.546, 5000.0
FAULT_BELOW_MA = 3.0


def as_signed(v: int) -> int:
    """Dien giai 16 bit co dau. revpimodio2 thuong da tra ve co dau san."""
    return v - 65536 if v > 32767 else v


def as_unsigned(v: int) -> int:
    return v + 65536 if v < 0 else v


def to_gram(mA: float, min_ma=None, max_ma=None) -> float:
    lo = MIN_MA if min_ma is None else min_ma
    hi = MAX_MA if max_ma is None else max_ma
    if mA < FAULT_BELOW_MA:
        return 0.0
    g = (mA - lo) / (hi - lo) * MAX_CAP_G
    return min(g, MAX_CAP_G)


def find_channel(rpi, pos, ch):
    dev = next((d for d in rpi.device if d.position == pos), None)
    if dev is None:
        sys.exit(f'khong thay module o vi tri {pos} '
                 f'(co: {[d.position for d in rpi.device]})')
    val = sta = None
    for io in dev.get_inputs():
        parts = io.name.split('_')
        if len(parts) >= 2 and parts[1] == str(ch):
            if parts[0] == 'InputValue':
                val = io
            elif parts[0] == 'InputStatus':
                sta = io
    if val is None:
        sys.exit(f'module pos {pos} khong co InputValue_{ch} '
                 f'(co: {[i.name for i in dev.get_inputs()]})')
    return dev, val, sta


def scan_all(rpi):
    rpi.readprocimg()
    print(f'{"module":>7} {"kenh":<18} {"raw":>8} {"co dau":>8} '
          f'{"khong dau":>10} {"mA":>8} {"status":>7}')
    print('-' * 72)
    for d in rpi.device:
        if 'AIO' not in d.name.upper():
            continue
        stat = {io.name: io.value for io in d.get_inputs()
                if io.name.startswith('InputStatus')}
        for io in d.get_inputs():
            if not io.name.startswith('InputValue'):
                continue
            raw = io.value
            s = as_signed(raw)
            st = stat.get(io.name.replace('InputValue', 'InputStatus'), '-')
            print(f'{d.position:>7} {io.name:<18} {raw:>8} {s:>8} '
                  f'{as_unsigned(raw):>10} {s / 1000.0:>8.3f} {st:>7}')


def watch(rpi, pos, ch, seconds, interval, min_ma=MIN_MA, max_ma=MAX_MA):
    dev, val_io, sta_io = find_channel(rpi, pos, ch)
    rpi.readprocimg()
    probe = val_io.value
    print(f'module pos {pos} ({dev.name}) — {val_io.name}')
    print(f'revpimodio2 tra ve: {probe}  -> dien giai '
          f'{"CO DAU (am duoc)" if probe < 0 else "duong / chua ket luan duoc"}')
    print(f'nguong FAULT: < {FAULT_BELOW_MA} mA | quy doi {min_ma}-{max_ma} mA '
          f'-> 0-{MAX_CAP_G:.0f} g  (phai khop tham so node!)\n')
    print(f'{"t(s)":>6} {"raw":>8} {"mA":>8} {"gram":>9} {"status":>7}  ghi chu')
    print('-' * 64)

    t0 = time.time()
    samples = []
    while time.time() - t0 < seconds:
        rpi.readprocimg()
        raw = val_io.value
        s = as_signed(raw)
        mA = s / 1000.0
        samples.append(s)
        note = ''
        if mA < 0:
            note = 'am -> khong co dong / dao cuc'
        elif mA < FAULT_BELOW_MA:
            note = 'duoi nguong FAULT'
        elif mA < min_ma:
            note = f'duoi {min_ma} mA -> duoi diem zero'
        elif mA > 20.5:
            note = 'vuot 20.5 -> qua dong / ho mach'
        st = sta_io.value if sta_io is not None else '-'
        print(f'{time.time() - t0:>6.1f} {raw:>8} {mA:>8.3f} '
              f'{to_gram(mA, min_ma, max_ma):>9.1f} {st:>7}  {note}')
        time.sleep(interval)

    lo, hi = min(samples), max(samples)
    print('-' * 64)
    print(f'so mau: {len(samples)}   min {lo} uA   max {hi} uA   '
          f'bien do {hi - lo} uA')
    # Cam bien song luon dao dong vai chuc uA. Phang tuyet doi = khong co
    # nguon dong, chi la offset zero cua ADC.
    if hi - lo <= 2:
        print('=> PHANG TUYET DOI: khong phai tin hieu song. Kenh dang khong '
              'co dong chay qua (offset ADC).')
    else:
        print(f'=> co dao dong {hi - lo} uA — kenh dang nhan tin hieu that.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pos', type=int, default=32, help='vi tri module AIO')
    ap.add_argument('--ch', type=int, default=4, help='so kenh InputValue')
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--interval', type=float, default=0.5)
    ap.add_argument('--all', action='store_true', help='quet moi kenh AIO')
    ap.add_argument('--min-ma', type=float, default=MIN_MA)
    ap.add_argument('--max-ma', type=float, default=MAX_MA,
                    help='mA ung voi day thang; phai khop max_current_mA cua node')
    a = ap.parse_args()

    # monitoring=True: chi doc, khong gianh quyen ghi voi loadcell_node dang chay
    rpi = revpimodio2.RevPiModIO(autorefresh=False, monitoring=True)
    if a.all:
        scan_all(rpi)
    else:
        watch(rpi, a.pos, a.ch, a.seconds, a.interval, a.min_ma, a.max_ma)


if __name__ == '__main__':
    main()
