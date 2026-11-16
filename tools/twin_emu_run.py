"""HIL adım 3: Uno ikiz emülatörü (arduino/twin_emu) + ECU kapalı çevrim testi, yalnız özet yazar.

ECU'da varsayılan yazılım (--app 4), Uno'da twin_emu olmalı:
    python tools/twin_emu_run.py [--dur 200] [--cmd "60:m 2" --cmd "120:m 0"]

Sıra: önce EMU (COM10 açılınca Uno sıfırlanır, ~2 s susar), sonra ECU SWD ile sıfırlanır.
--cmd "t:satır": t saniyede Uno'ya seri satır gönderir (m <mod>, s <soc>, a <°C>, c <cmd> <val>).
Tüm R satırları build/twin_emu_run.csv'ye yazılır. PASS: ilk 10 s sonrası ECU hep NORMAL,
CAN hata sayaçları (iki taraf TEC/REC, CRC/sayaç hataları, ECU'nun hücre çerçevesi hataları) 0,
0x210 alındı, ikiz SOC değişti. --bal X: SOC değişimi yerine dengeleme açıldı ve gruplar arası
SOC farkı en az X puan azaldı (ör. bekleme modunda hızlandırılmış dengeleme:
    --bal 0.3 --cmd "1:m 3" --cmd "1:b 500").
"""
import argparse
import glob
import os
import re
import subprocess
import threading
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--ecu", default="COM9")
ap.add_argument("--emu", default="COM10")
ap.add_argument("--dur", type=float, default=200.0)
ap.add_argument("--cmd", action="append", default=[])
ap.add_argument("--bal", type=float, default=None,
                help="dengeleme testi: SOC farkı en az bu kadar [%%] azalmalı (ikiz SOC'si değişmeyebilir)")
a = ap.parse_args()

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
CSV = os.path.join(ROOT, "build", "twin_emu_run.csv")
HDR = ("R,t,mode,v_kmh,I,V,soc,Tc,Ts,Ta,ta_src,state,Idis,Ichg,cause,soc_ecu,tc_ecu,"
       "ok201,ok202,err,TEC,REC,bal_mask,bal_flags,dv_ecu_mV,dv_mV,dsoc,bal_k,ok210,step_us,faults")
TEC = re.compile(r"TEC (\d+) REC (\d+)")
CELL = re.compile(r"cell:(\d+)(?:\(err (\d+)\))?")
ecu_tec = [0, 0]
ecu_cell = [0, 0]          # ECU'nun aldığı hücre çerçevesi, hata (son satır)
ecu_last = [""]
stop = False


def ecu_reader(s):
    buf = b""
    while not stop:
        buf += s.read(1024)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            t = ln.decode(errors="replace").strip()
            m = TEC.search(t)
            if m:
                ecu_tec[0] = max(ecu_tec[0], int(m.group(1)))
                ecu_tec[1] = max(ecu_tec[1], int(m.group(2)))
                ecu_last[0] = t
                c = CELL.search(t)
                if c:
                    ecu_cell[0], ecu_cell[1] = int(c.group(1)), int(c.group(2) or 0)


cmds = sorted((float(c.split(":", 1)[0]), c.split(":", 1)[1]) for c in a.cmd)
cli = sorted(glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe"))[-1]
emu = serial.Serial(a.emu, 115200, timeout=0.2)
time.sleep(3.0)
banner = emu.read(emu.in_waiting or 1).decode(errors="replace")
ecu = serial.Serial(a.ecu, 115200, timeout=0.2)
threading.Thread(target=ecu_reader, args=(ecu,), daemon=True).start()
subprocess.run([cli, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True)
t0 = time.time()

rows, buf, other = [], b"", []
cell_err0 = None
with open(CSV, "w") as f:
    f.write(HDR + "\n")
    while time.time() - t0 < a.dur:
        el = time.time() - t0
        while cmds and el >= cmds[0][0]:
            emu.write((cmds.pop(0)[1] + "\n").encode())
        buf += emu.read(512)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            t = ln.decode(errors="replace").strip()
            if t.startswith("R,"):
                f.write(t + "\n")
                p = t.split(",")
                if len(p) >= 30:
                    rows.append(p)
                    if cell_err0 is None and int(p[1]) >= int(rows[0][1]) + 10:
                        cell_err0 = ecu_cell[1]          # açılış (sıfırlama geçişi) hariç
            elif t:
                other.append(t)
stop = True
time.sleep(0.3)

for ln in (banner.strip().splitlines() + other)[:8]:
    print("EMU:", ln)
if not rows:
    print("FAIL: Uno'dan R satırı gelmedi")
    raise SystemExit(1)

t_ = [int(r[1]) for r in rows]
num = lambda i: [float(r[i]) for r in rows]
st, soc, soce, tc, tce, I, err = num(11), num(6), num(15), num(7), num(16), num(4), num(19)
tec, rec = num(20), num(21)
t_start = t_[0] + 10
steady = [(t, s, r[14]) for t, s, r in zip(t_, st, rows) if t >= t_start]
states = {}
for _, s, _c in steady:
    states[int(s)] = states.get(int(s), 0) + 1
# --bal: beklemede en yüksek grup şarj payı bandına girip kısa DERATE verebilir (cause 8, gerçek
# şarj sınırı); dengeleme testinde yalnız LIMP / SAFE hata sayılır
bad = [(t, int(s), c) for t, s, c in steady if (s not in (1, 2) if a.bal is not None else s != 1)]
modes = sorted(set(r[2] for r in rows))
late = [i for i, t in enumerate(t_) if t >= t_[0] + 60]
dsoc = max(abs(soc[i] - soce[i]) for i in late) if late else float("nan")
dtc = max(abs(tc[i] - tce[i]) for i in late) if late else float("nan")

print(f"sure {t_[-1] - t_[0]} s, satir {len(rows)}, mod {modes}, Ta kaynak {rows[-1][10]} {rows[-1][9]} C")
print(f"ECU durum (t>{t_start}s) {states}" + (f", ilk NORMAL disi: t={bad[0][0]} durum {bad[0][1]} neden {bad[0][2]}" if bad else ""))
print(f"I min/max {min(I):.1f}/{max(I):.1f} A, Idis son {rows[-1][12]} Ichg son {rows[-1][13]}")
print(f"SOC ikiz {soc[0]:.2f} -> {soc[-1]:.2f} %, ECU {soce[0]:.2f} -> {soce[-1]:.2f} %, max fark (t>60s) {dsoc:.2f} %")
print(f"Tc ikiz {min(tc):.2f}..{max(tc):.2f} C, ECU tc_hat son {tce[-1]:.2f}, max fark (t>60s) {dtc:.2f} C")
# açılış (ECU sıfırlanırken sayaç kopması, ACK'sız çerçeveler) hariç: ilk 10 s sonrası
k0 = next((i for i, t in enumerate(t_) if t >= t_start), len(t_) - 1)
derr = err[-1] - err[k0]
tec_m, rec_m = max(tec[k0:]), max(rec[k0:])
print(f"CAN (t>{t_start}s): Uno yeni hata {int(derr)}, Uno TEC/REC max {int(tec_m)}/{int(rec_m)}, "
      f"ECU TEC/REC max (tum test) {ecu_tec[0]}/{ecu_tec[1]}")
nbal = [bin(int(r[22], 16)).count("1") for r in rows]
dv, dvecu, dsoc = num(25), num(24), num(26)
first_bal = next((t for t, n in zip(t_, nbal) if n), None)
cell_derr = ecu_cell[1] - (cell_err0 or 0)
print(f"hucre: ECU aldi {ecu_cell[0]} cerceve, hata {ecu_cell[1]} (t>{t_start}s: {cell_derr}); 0x210 alinan {rows[-1][28]}; "
      f"Uno adim max {max(int(r[29]) for r in rows)} us; dengeleme carpani x{rows[-1][27]}")
print(f"dengeleme: ilk acilis t={first_bal} s, en cok {max(nbal)} direnc acik; dV ikiz {dv[0]:.1f} -> {dv[-1]:.1f} mV "
      f"(ECU {dvecu[-1]:.1f}), SOC farki {dsoc[0]:.3f} -> {dsoc[-1]:.3f} %, ECU bayrak son {rows[-1][23]}")
ok = (not bad and derr == 0 and tec_m == 0 and rec_m == 0 and cell_derr == 0 and int(rows[-1][28]) > 0
      and (a.bal is None and abs(soc[-1] - soc[0]) > 0.05 or a.bal is not None and first_bal is not None
           and dsoc[-1] < dsoc[0] - a.bal))
print("PASS" if ok else "FAIL", f"(CSV: {os.path.relpath(CSV, ROOT)})")
