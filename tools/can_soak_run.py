"""
Adım 1, 10 dakikalık CAN dayanıklılık testinin çalıştırıcısı.
STM32: APP_CAN_SOAK (python tools/fw_build.py --app 5 --flash), Arduino: arduino/can_soak yüklü.
Sıra: STM32 ST-LINK ile sıfırlanır, 2 s sonra Arduino'nun portu açılır (Uno sıfırlanır, gönderim
başlar). STM32 (COM9) ve Arduino (COM10) satırları ekrana ve log dosyasına yazılır; STM32'nin
RESULT tablosu gelince biter (en çok 660 s).
Kullanım:  python tools/can_soak_run.py soak.log [--stm COM9] [--ard COM10]
"""
import argparse, glob, subprocess, sys, threading, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--stm", default="COM9")
ap.add_argument("--ard", default="COM10")
args = ap.parse_args()
log_path = args.log
t0 = time.time()
lines, stop = [], False
lock = threading.Lock()


def reader(port, tag):
    s = serial.Serial(port, 115200, timeout=0.3)
    buf = b""
    while not stop:
        buf += s.read(1024)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            with lock:
                lines.append((time.time() - t0, tag, ln.decode(errors="replace").rstrip()))
            print(f"{time.time() - t0:7.1f} {tag} {ln.decode(errors='replace').rstrip()}", flush=True)
    s.close()


# Sıra: önce alıcı (STM32) hazır olsun, sonra gönderen (Uno, port açılınca sıfırlanır) başlasın;
# aksi hâlde STM32 açılırken ACK alamayan çerçeveler Uno'nun tamponunda bekleyip sırasız gelir.
threading.Thread(target=reader, args=(args.stm, "STM"), daemon=True).start()
cli = sorted(glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe"))[-1]
subprocess.run([cli, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True)
time.sleep(2.0)
threading.Thread(target=reader, args=(args.ard, "ARD"), daemon=True).start()
deadline = time.time() + 660
while time.time() < deadline:
    time.sleep(1)
    with lock:
        if sum(1 for _, g, l in lines if g == "STM" and l.startswith("RESULT")) >= 2:
            break
time.sleep(1)
stop = True
time.sleep(0.5)
with open(log_path, "w", encoding="utf-8") as f:
    for t, g, l in lines:
        f.write(f"{t:7.1f} {g} {l}\n")
