#!/usr/bin/env python3
"""Can chinh thang do loadcell bang NHIEU DIEM — chay TREN RevPi.

Vi sao can: can chinh bang tay qua hai diem roi nham tay de sai, va moi lan POT
SPAN bi van thi con so cu thanh vo nghia. Cong cu nay do nhieu diem, khop duong
thang bang binh phuong toi thieu, roi in ra dung tham so can dat — kem sai so
tung diem de biet chuoi co that su tuyen tinh khong.

Chay:
    python3 loadcell_cal.py                    # nhap tai bang tay tung buoc
    python3 loadcell_cal.py 0 100 300 2000     # doc san danh sach tai (gam)

Voi moi diem, dat tai roi Enter. Cong cu doi so on dinh truoc khi ghi.
Khong ghi gi vao he thong — chi in ra lenh de ban tu ap dung.
"""

import sys
import time
import warnings

warnings.filterwarnings('ignore')

try:
    import revpimodio2
except ImportError:
    sys.exit('khong import duoc revpimodio2 — phai chay tren RevPi')

AIO_POS = 32
CHANNEL = 4
MAX_CAP_G = 5000.0        # phai khop max_capacity_g cua node
SETTLE_TOL_UA = 3         # coi la on dinh khi bien do <= nguong nay
SETTLE_SAMPLES = 6


def signed(v):
    return v - 65536 if v > 32767 else v


def read_settled(rpi, io):
    """Doc cho toi khi so dung yen, tra ve uA trung binh."""
    buf = []
    print('    doi so on dinh', end='', flush=True)
    for _ in range(120):                       # toi da ~24s
        rpi.readprocimg()
        buf.append(signed(io.value))
        if len(buf) > SETTLE_SAMPLES:
            buf.pop(0)
            if max(buf) - min(buf) <= SETTLE_TOL_UA:
                break
        print('.', end='', flush=True)
        time.sleep(0.2)
    avg = sum(buf) / len(buf)
    print(f' -> {avg:.0f} uA ({avg/1000:.3f} mA)')
    return avg


def fit(points):
    """Binh phuong toi thieu: uA = a * gam + b. Tra ve (a, b)."""
    n = len(points)
    sx = sum(g for g, _ in points)
    sy = sum(u for _, u in points)
    sxx = sum(g * g for g, _ in points)
    sxy = sum(g * u for g, u in points)
    den = n * sxx - sx * sx
    if abs(den) < 1e-9:
        sys.exit('cac diem tai trung nhau — can it nhat hai muc tai khac nhau')
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    return a, b


def main():
    loads = [float(x) for x in sys.argv[1:]] if len(sys.argv) > 1 else None

    rpi = revpimodio2.RevPiModIO(autorefresh=False, monitoring=True)
    dev = next((d for d in rpi.device if d.position == AIO_POS), None)
    if dev is None:
        sys.exit(f'khong thay AIO o vi tri {AIO_POS}')
    io = next(i for i in dev.get_inputs()
              if i.name.split('_')[0] == 'InputValue'
              and i.name.split('_')[1] == str(CHANNEL))

    print(f'AIO pos {AIO_POS}, {io.name}, tam can {MAX_CAP_G:.0f} g')
    print('Bat dau bang diem 0 (can TRONG), roi cac muc tai biet truoc.\n')

    points = []
    if loads is None:
        while True:
            s = input('Khoi luong dat len (gam), Enter suong de ket thuc: ').strip()
            if not s:
                break
            try:
                g = float(s)
            except ValueError:
                print('  khong phai so, thu lai'); continue
            points.append((g, read_settled(rpi, io)))
    else:
        for g in loads:
            input(f'Dat {g:.0f} g len can roi Enter... ')
            points.append((g, read_settled(rpi, io)))

    if len(points) < 2:
        sys.exit('can it nhat hai diem')

    a, b = fit(points)                          # uA = a*gam + b
    min_ma = b / 1000.0
    max_ma = (b + a * MAX_CAP_G) / 1000.0

    print('\n' + '=' * 62)
    print(f'do doc      = {a:.4f} uA/gam')
    print(f'diem zero   = {b:.0f} uA = {min_ma:.3f} mA')
    print(f'day thang   = {max_ma:.3f} mA cho {MAX_CAP_G:.0f} g')
    print('=' * 62)

    print(f'\n{"tai (g)":>9} {"do (uA)":>9} {"khop (uA)":>10} {"lech (g)":>9}')
    worst = 0.0
    for g, u in points:
        fitted = a * g + b
        err_g = (u - fitted) / a
        worst = max(worst, abs(err_g))
        print(f'{g:>9.0f} {u:>9.0f} {fitted:>10.0f} {err_g:>+9.1f}')

    print(f'\nsai so tuyen tinh lon nhat: {worst:.1f} g')
    if worst > MAX_CAP_G * 0.005:
        print('  => LON. Chuoi khong tuyen tinh: nghi co khi (ban can vuong, ke')
        print('     khong phang, dat lech tam) hoac qua can khong chuan.')
    else:
        print('  => tot, chuoi tuyen tinh.')

    if min_ma < 3.5:
        print(f'\nCANH BAO: diem zero {min_ma:.3f} mA qua thap. Nguong mat tin hieu')
        print('  cua node la 3.0 mA — van POT Zero len de con du bien an toan.')
    if max_ma > 20.0:
        cap = (20.15 - min_ma) / (max_ma - min_ma) * MAX_CAP_G
        print(f'\nLUU Y: day thang {max_ma:.3f} mA vuot dai 20 mA cua AIO.')
        print(f'  Chi doc duoc toi khoang {cap:.0f} g. Van POT SPAN xuong neu can')
        print(f'  can toi sat {MAX_CAP_G:.0f} g.')

    print('\n--- Ap dung: sua /home/pi/start_loadcell.sh roi restart node ---')
    print(f"sed -i 's|-p min_current_mA:=[0-9.]*||; s|-p max_current_mA:=[0-9.]*||' "
          f"/home/pi/start_loadcell.sh")
    print(f"sed -i 's|-p use_simulation:=false|-p use_simulation:=false "
          f"-p min_current_mA:={min_ma:.3f} -p max_current_mA:={max_ma:.3f}|' "
          f"/home/pi/start_loadcell.sh")
    print("tmux kill-session -t loadcell; sleep 2; "
          "tmux new-session -d -s loadcell "
          "'exec bash /home/pi/start_loadcell.sh > /tmp/loadcell_node.log 2>&1'")
    print('\nSau khi restart: TRU BI voi can trong (restart xoa tru bi cu).')


if __name__ == '__main__':
    main()
