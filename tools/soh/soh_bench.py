"""Capacity SOH on the bench (docs/soh_design.md, validation 3 + 4). ECU image: APP_ECU (any flags).

    python tools/soh/soh_bench.py persist          # ~3 min: record survives reset / reflash / torn write
    python tools/soh/soh_bench.py learn [--soh 85] # ~55 min: twin aged pack, park 20 min, drive, park 20 min

persist: erases NVM pages 61-62 (CLI), boots ("no record"), sends 0x301 NEW_PACK (cmd 0x05, unlock) through
  twin_emu -> record #1; resets -> must load #1; reflashes the same image -> must still load #1; writes a
  garbage record into the next slot (bad CRC, like a torn write) -> must still load #1 and report 1 bad slot.
learn: twin `y <soh>` + park; waits for the ECU rest point (anchor), drives at full pedal until the twin SOC
  fell >= 35 points, parks again and waits for the next rest point (expects ACCEPTED, ratio ~ twin SOH),
  then resets the ECU and checks the boot line carries the learned capacity.
Order: EMU first (opening COM10 resets the Uno, ~2 s), then ECU reset over SWD. Logs: build/soh/bench_<phase>.log.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, "build", "soh")
CLI = sorted(glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe"))[-1]
NVM = 0x0801E800

t0 = time.time()
log, lock = [], threading.Lock()
stop = False


def reader(port, tag):
    buf = b""
    while not stop:
        buf += port.read(4096)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            with lock:
                log.append((time.time() - t0, tag, ln.decode(errors="replace").rstrip()))


def wait_for(tag, pattern, timeout, start=0):
    """Returns (index, match) of the first log line from `tag` at/after index `start` matching `pattern`."""
    rx = re.compile(pattern)
    end = time.time() + timeout
    while time.time() < end:
        with lock:
            for k in range(start, len(log)):
                if log[k][1] == tag:
                    m = rx.search(log[k][2])
                    if m:
                        return k, m
        time.sleep(0.2)
    return None, None


def mark():
    with lock:
        return len(log)


def note(text):
    with lock:
        log.append((time.time() - t0, "PC ", text))
    print(f"[{time.time() - t0:7.1f} s] {text}", flush=True)


def cli(*args):
    return subprocess.run([CLI, "-c", "port=SWD", "mode=UR", *args], capture_output=True, text=True)


def ecu_reset():
    subprocess.run([CLI, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True)


def save(phase):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"bench_{phase}.log")
    with lock, open(path, "w", encoding="utf-8") as f:
        for t, g, l in log:
            f.write(f"{t:8.1f} {g} {l}\n")
    return path


def boot_line(k0, timeout=15):
    _, m = wait_for("ECU", r"^soh: (.*)$", timeout, k0)
    return m.group(1) if m else None


def open_ports():
    emu = serial.Serial("COM10", 115200, timeout=0.2)
    threading.Thread(target=reader, args=(emu, "EMU"), daemon=True).start()
    time.sleep(3.0)
    ecu = serial.Serial("COM9", 115200, timeout=0.2)
    threading.Thread(target=reader, args=(ecu, "ECU"), daemon=True).start()
    return emu, ecu


def persist(elf):
    results = []
    r = cli("-e", "[61 62]")
    note(f"NVM pages 61-62 erased: {'OK' if r.returncode == 0 else 'FAIL'}")
    emu, _ = open_ports()
    emu.write(b"m 3\n")                                # park: NEW_PACK yalnız |I| < 1 A'de kabul
    k = mark(); ecu_reset()
    b = boot_line(k); note(f"boot 1: {b}")
    results.append(("fresh boot reports no record", b is not None and "no record" in b))
    time.sleep(3.0)                                    # 0x301 boot hold-off (1 s) + fresh 0x101
    k = mark(); emu.write(b"c 5 0xA5A5\n")
    _, m = wait_for("ECU", r"cmd 0x05 .*: (OK|REJECTED)(.*)", 5, k)
    note(f"NEW_PACK: {m.group(0) if m else 'no answer'}")
    results.append(("NEW_PACK accepted and record written", bool(m) and m.group(1) == "OK"))
    k = mark(); ecu_reset()
    b = boot_line(k); note(f"boot 2 (reset): {b}")
    results.append(("record #1 loaded after reset", b is not None and "record #1 " in b))
    k = mark()
    r = cli("-w", elf, "-v", "-rst")
    b = boot_line(k, 30); note(f"boot 3 (reflash): {b}")
    results.append(("record #1 survives reflash", r.returncode == 0 and b is not None and "record #1 " in b))
    # torn write: magic + seq 99 + garbage, wrong CRC, in slot 1 (record #1 sits in slot 0 after erase)
    junk = os.path.join(OUT, "torn_record.bin")
    with open(junk, "wb") as f:
        f.write((0x534F4831).to_bytes(4, "little") + (99).to_bytes(4, "little") + bytes(range(40)))
    k = mark()
    r = cli("-w", junk, hex(NVM + 48), "--skipErase", "-rst")   # sayfayı silmeden (kayıt #1 kalsın)
    b = boot_line(k, 30); note(f"boot 4 (torn record in slot 1): {b}")
    results.append(("torn record ignored, #1 still loaded", b is not None and "record #1 " in b and "bad 1" in b))
    return results


def learn(soh):
    results = []
    emu, _ = open_ports()
    emu.write(f"y {soh}\n".encode()); time.sleep(0.3)
    emu.write(b"m 3\n")
    note(f"twin SOH {soh} %, park")
    time.sleep(1.0)
    k = mark(); ecu_reset()
    note(f"boot: {boot_line(k)}")
    # 1. dinlenme noktası (çapa)
    i, m = wait_for("ECU", r"^soh: rest point SOC ([\d.]+) % -> (\S.*?)( \||$)", 25 * 60, k)
    note(f"rest point 1: {log[i][2] if i is not None else 'none'}")
    if i is None:
        return [("first rest point (20 min park)", False)]
    soc_a = float(m.group(1))
    results.append(("first rest point taken (anchor or small)", True))
    # 2. sürüş: tam pedal, ikiz SOC 35 puan düşene kadar
    emu.write(b"p 100\n")
    note(f"drive: full pedal from ECU rest SOC {soc_a:.1f} %")
    k = mark()
    soc0 = None
    while True:
        _, mm = wait_for("EMU", r"^R,\d+,[A-Z]+,[-\d.]+,[-\d.]+,[-\d.]+,([-\d.]+),", 30, k)
        k = mark()
        if not mm:
            note("no twin report"); break
        s = float(mm.group(1))
        soc0 = s if soc0 is None else soc0
        if soc0 - s >= 35.0 or s < 15.0:
            break
        time.sleep(5)
    emu.write(b"m 3\n")
    note(f"park: twin SOC {s:.1f} % (drop {soc0 - s:.1f} points)")
    # 3. ikinci dinlenme noktası: ölçüm
    i, m = wait_for("ECU", r"^soh: rest point SOC ([\d.]+) % -> ACCEPTED ratio ([\d.]+)%.*capacity ([\d.]+)% shown ([\d.]+)%",
                    25 * 60, k)
    note(f"rest point 2: {log[i][2] if i is not None else 'none'}")
    results.append(("capacity measurement ACCEPTED", i is not None))
    if i is not None:
        ratio, cap = float(m.group(2)), float(m.group(3))
        results.append((f"measured ratio {ratio:.1f} % within 3 points of twin SOH {soh} %", abs(ratio - soh) <= 3.0))
        k = mark(); ecu_reset()
        b = boot_line(k); note(f"boot after learning: {b}")
        results.append((f"learned capacity {cap:.1f} % reloaded at boot",
                        b is not None and f"capacity {cap:.1f}%" in b))
    emu.write(b"m 0\n")                                # sürüş profiline geri
    return results


def main():
    global stop
    ap = argparse.ArgumentParser()
    ap.add_argument("phase", choices=["persist", "learn"])
    ap.add_argument("--soh", type=int, default=85)
    ap.add_argument("--elf", default=os.path.join(ROOT, "build", "app4_O2", "bms_ecu.elf"))
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    res = persist(a.elf) if a.phase == "persist" else learn(a.soh)
    stop = True
    time.sleep(0.5)
    path = save(a.phase)
    print("\nRESULT")
    for name, ok in res:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    print("log:", path)
    return 0 if res and all(ok for _, ok in res) else 1


if __name__ == "__main__":
    sys.exit(main())
