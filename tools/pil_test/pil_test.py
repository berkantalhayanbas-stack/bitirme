"""
PIL (processor-in-the-loop) testi: BMS firmware'i GERÇEK STM32G431 üzerinde, ölçümler PC'den.

PC, kayıtlı CAN ölçüm dizisini (data/<senaryo>_in.csv) ST-LINK sanal COM portundan karta yollar;
kart her ölçüm için bms_step'i çalıştırır, sonucu ve süresini (DWT döngü sayısı) döndürür.
Sonuçlar PC'de derlenmiş aynı C kodunun çıktısıyla (data/<senaryo>_c.csv, SIL) karşılaştırılır.

Kabul:  SOC farkı < 0.05 puan, durum dizisi birebir;  bms_step < 1 ms (hedef < 200 µs).

Kullanım:
  pip install pyserial pandas numpy
  python pil_test.py --port COM5                 # gerçek kart (Aygıt Yöneticisi > Bağlantı noktaları)
  python pil_test.py --port COM5 --n 3000        # hızlı deneme: ilk 3000 örnek
  python pil_test.py --sim                       # kart olmadan, PC'deki "sanal kart" ile protokol testi
"""
import argparse
import os
import struct
import subprocess
import sys
import time
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
F_CPU = 170e6


def crc8_j1850(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def frame(cmd, vals):
    body = struct.pack("<B6f", cmd, *vals)
    return b"\xA5\x5A" + body + bytes([crc8_j1850(body)])


class SerialLink:
    def __init__(self, port, baud):
        import serial
        self.s = serial.Serial(port, baud, timeout=2)
        time.sleep(0.3)
        self.s.reset_input_buffer()            # açılış mesajı "BMS PIL ready" atılır

    def xfer(self, data, n):
        self.s.write(data)
        r = self.s.read(n)
        if len(r) != n:
            raise TimeoutError("no answer from the board (port, baud rate, firmware?)")
        return r


class SimLink:                                  # PC'de derlenmiş app_pil.c (protokol testi)
    def __init__(self):
        exe = os.path.join(HERE, "hoststub", "virtual_board" + (".exe" if os.name == "nt" else ""))
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.p.stdout.read(len(b"BMS PIL ready\r\n"))

    def xfer(self, data, n):
        self.p.stdin.write(data)
        self.p.stdin.flush()
        r = b""
        while len(r) < n:
            r += self.p.stdout.read(n - len(r))
        return r


def parse(r):
    if r[:2] != b"\x5A\xA5" or crc8_j1850(r[2:47]) != r[47]:
        raise ValueError("corrupted reply")
    status = r[2]
    f = struct.unpack("<9f", r[3:39])
    state, cause, flags, cycles = r[39], struct.unpack("<H", r[40:42])[0], r[42], struct.unpack("<I", r[43:47])[0]
    return status, f, state, cause, flags, cycles


def run(link, scenario, n_max):
    inp = pd.read_csv(os.path.join(HERE, "data", f"{scenario}_in.csv"))
    ref = pd.read_csv(os.path.join(HERE, "data", f"{scenario}_c.csv"))
    n = len(inp) if n_max <= 0 else min(n_max, len(inp))
    st, *_ = parse(link.xfer(frame(1, [0.5, 0, 0, 0, 0, 0]), 48))
    assert st == 2, "reset not acknowledged"
    out = np.zeros((n, 9)); state = np.zeros(n, int); cause = np.zeros(n, int); cyc = np.zeros(n)
    t0 = time.time()
    for k in range(n):
        st, f, state[k], cause[k], _, cyc[k] = parse(link.xfer(frame(0, inp.iloc[k].values), 48))
        if st != 0:
            raise ValueError(f"board reported CRC error at sample {k}")
        out[k] = f
        if (k + 1) % 2000 == 0:
            print(f"  {scenario}: {k + 1}/{n} samples ({time.time() - t0:.0f} s)")
    r = ref.iloc[:n]
    us = cyc / F_CPU * 1e6
    return dict(Scenario=scenario, Samples=n,
                SOC_maxdiff_pts=100 * np.max(np.abs(out[:, 0] - r.soc.values)),
                Tcore_maxdiff_C=np.max(np.abs(out[:, 2] - r.tc.values)),
                Iallow_maxdiff_A=np.max(np.abs(out[:, 6] - r.i_dis.values)),
                State_mismatch=int(np.sum(state != r.state.values)),
                Cause_mismatch=int(np.sum(cause != r.cause.values)),
                Step_mean_us=us.mean(), Step_max_us=us.max(), Cycles_max=int(cyc.max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--n", type=int, default=0, help="samples per scenario (0 = all)")
    ap.add_argument("--scenarios", default="S6_ntc_open,S3_deep_weak")
    a = ap.parse_args()
    if not a.sim and not a.port:
        sys.exit("give --port COMx (or --sim)")
    link = SimLink() if a.sim else SerialLink(a.port, a.baud)
    rows = [run(link, s, a.n) for s in a.scenarios.split(",")]
    R = pd.DataFrame(rows)
    pd.set_option("display.width", 200)
    target = "virtual board (PC)" if a.sim else "STM32G431 @ 170 MHz"
    print(f"\n=== PIL: {target} vs PC SIL (same C code, same CAN inputs) ===")
    print(R.to_string(index=False, float_format=lambda v: f"{v:.3g}"))
    ok1 = (R.SOC_maxdiff_pts < 0.05).all() and (R.State_mismatch == 0).all()
    print(f"Acceptance 1 (SOC diff < 0.05 pts, identical states): {'PASS' if ok1 else 'FAIL'}")
    if not a.sim:
        ok2 = (R.Step_max_us < 1000).all()
        print(f"Acceptance 2 (bms_step < 1 ms; goal < 200 us): {'PASS' if ok2 else 'FAIL'} "
              f"(max {R.Step_max_us.max():.1f} us, mean {R.Step_mean_us.mean():.1f} us)")
    R.to_csv(os.path.join(HERE, "pil_results_sim.csv" if a.sim else "pil_results.csv"), index=False)


if __name__ == "__main__":
    main()
