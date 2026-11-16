"""Far / kontaktör çıkışlarının yavaş gösterimi: NORMAL -> LIMP -> SAFE.

ECU'da BMS_OUTPUTS=1 yazılımı, Arduino'da mcp2515_node olmalı:
    python tools/fw_build.py --app 4 --define BMS_OUTPUTS=1 --flash
    python tools/out_demo.py [--normal 30] [--limp 15] [--safe 10]

Sıra: önce EMU (COM10 açılınca Uno sıfırlanır, ~2 s susar), sonra ECU sıfırlanır.
Her saniye durum, far dolulukları ve kontaktör ekrana yazılır; aşama
geçişleri büyük başlıkla gösterilir. Akım izni rampası (10 A/s) algoritmanın
parçası, burada değiştirilmez; yavaşlatma aşama sürelerini uzatarak yapılır.
"""
import argparse
import glob
import re
import subprocess
import threading
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--ecu", default="COM9")
ap.add_argument("--emu", default="COM10")
ap.add_argument("--normal", type=float, default=30.0, help="NORMAL süresi [s]")
ap.add_argument("--limp", type=float, default=15.0, help="LIMP süresi [s]")
ap.add_argument("--safe", type=float, default=10.0, help="SAFE süresi [s]")
a = ap.parse_args()

PAT = re.compile(r"t=(\d+)s (\w+).*?Idis\s+([\d.]+).*?OUT low set ([\d.]+)%.*?high set ([\d.]+)%.*?K (\d)(?: \w+ pin (\d))?")
TEC = re.compile(r"TEC (\d+) REC (\d+) LEC \d+ ([A-Z-]+)")   # CAN hata sayaçları: besleme / toprak gürültüsünün izi
stop = False


def bar(p):
    n = int(round(float(p) / 5.0))
    return "#" * n + "." * (20 - n)


def ecu_reader(s):
    buf = b""
    while not stop:
        buf += s.read(1024)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            m = PAT.search(ln.decode(errors="replace"))
            if m:
                t, st, idis, lo, hi, k, pin = m.groups()
                ce = TEC.search(ln.decode(errors="replace"))
                can = f"  CAN TEC {ce.group(1):>3} REC {ce.group(2):>3} {ce.group(3)}" if ce else ""
                print(f"  t={int(t):3d}s  {st:<6}  I_izin {float(idis):5.1f} A   "
                      f"kisa [{bar(lo)}] {float(lo):5.1f}%   uzun [{bar(hi)}] {float(hi):5.1f}%   "
                      f"role {'CEKTI (yesil yanar)' if k == '1' else 'BIRAKTI (sonuk)'}"
                      + (f"  PB5={pin}" if pin else "") + can, flush=True)


def banner(text):
    print("\n" + "=" * 70 + f"\n  {text}\n" + "=" * 70, flush=True)


cli = sorted(glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe"))[-1]
emu = serial.Serial(a.emu, 115200, timeout=0.2)
print("EMU açıldı, Uno'nun açılması bekleniyor...", flush=True)
time.sleep(3.0)
ecu = serial.Serial(a.ecu, 115200, timeout=0.2)
threading.Thread(target=ecu_reader, args=(ecu,), daemon=True).start()
subprocess.run([cli, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True)

banner(f"NORMAL ({a.normal:.0f} s): kısa far kısıktan tam parlaklığa çıkar (~9 s), uzun far izin %70'i geçince yanar")
time.sleep(a.normal)
banner(f"LIMP ({a.limp:.0f} s): NTC kopuk -> kısa far iyice kısılır (duty %3), uzun far söner, röle çekili")
emu.write(f"c 0x14 {int(a.limp * 1000)}\n".encode())
time.sleep(a.limp)
banner(f"SAFE ({a.safe:.0f} s): CAN sessiz -> farlar söner, röle bırakır")
emu.write(b"c 0x13 1000\n")
time.sleep(a.safe)
stop = True
time.sleep(0.3)
print("\nBitti.", flush=True)
