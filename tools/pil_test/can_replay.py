"""
CAN oynatma testi: kayıtlı ölçüm dizisi GERÇEK CAN hattından ECU'ya verilir, ECU'nun CAN'dan
döndürdüğü sonuçlar PC'deki SIL çıktısıyla örnek örnek karşılaştırılır.

  PC (bu betik) --seri--> Arduino + MCP2515 (arduino/mcp2515_bridge) --CAN 0x101/0x102-->
      STM32G431 + TJA1050 (APP_ECU: bms_step) --CAN 0x201/0x202--> Arduino --seri--> PC

PIL testinden farkı: ölçümler ve sonuçlar UART yerine CAN çerçeveleriyle taşınır (bms.dbc
ölçekleri, alive counter, CRC), yani kodlama/çözme, FDCAN sürücüsü, transceiver ve ECU
uygulamasının tamamı zincirin içindedir. Kilitli adım: her örnek için ECU tam bir bms_step atar.

Karşılaştırma: sonuçlar CAN'da ölçeklenip yuvarlandığı için fark yarım LSB kadar olabilir
(SOC 0.01 puan, Tc 0.1 °C, I_allow 0.1 A, psi 0.005). Üstüne kart ile PC arasındaki küçük
kayan nokta farkı eklenir (-O2'de FMA; PIL testinde SOC'de ~6e-4 puan): tolerans = 0.5 LSB +
ALGO payı. Durum ve neden kodu birebir olmalı.

Kullanım:
  pip install pyserial pandas numpy
  python can_replay.py --port COM10                # Arduino köprüsünün portu
  python can_replay.py --port COM10 --n 3000       # hızlı deneme: ilk 3000 örnek
Her senaryodan önce ECU, ST-LINK üzerinden sıfırlanır (STM32_Programmer_CLI). --no-reset ile
kapatılır; o zaman ECU'yu elle sıfırlayıp (siyah düğme) 1-2 s içinde betiği başlatın.
"""
import argparse
import glob
import os
import struct
import subprocess
import sys
import time
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = {1: "NORMAL", 2: "DERATE", 3: "LIMP", 4: "SAFE"}
ST_CTRL, ST_STAT, ST_SER_CRC, ST_TX_ERR, ST_STALE, ST_CAN_CRC, ST_HELLO = 1, 2, 4, 8, 16, 32, 128
# CAN ölçekleri (bms_can.c) ve kart/PC kayan nokta farkı payı (fiziksel birimde).
# SOC payı PIL kabul sınırıyla aynı (0.05 puan); diğerleri PIL'de ölçülen farkların çok üstünde.
LSB = dict(soc_pts=0.01, tc=0.1, i_dis=0.1, i_chg=0.1, psi=0.005)
ALGO = dict(soc_pts=0.05, tc=0.01, i_dis=0.01, i_chg=0.01, psi=0.001)


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


class Bridge:
    def __init__(self, port, baud):
        import serial
        self.s = serial.Serial(port, baud, timeout=1.0)
        time.sleep(2.0)                          # port açılınca Uno sıfırlanır; açılışı bekle
        self.s.reset_input_buffer()
        for _ in range(5):                       # merhaba: köprü sketch'i yüklü mü?
            try:
                st = self.xfer(frame(1, [0] * 6))[0]
                if st & ST_HELLO:
                    return
            except (TimeoutError, ValueError):
                self.s.reset_input_buffer()
        sys.exit("no answer from the bridge: is arduino/mcp2515_bridge uploaded, is the Serial Monitor closed?")

    def xfer(self, data):
        self.s.write(data)
        r = self.s.read(30)
        if len(r) != 30:
            raise TimeoutError("no answer from the bridge")
        if r[:2] != b"\x5A\xA5" or crc8_j1850(r[2:29]) != r[29]:
            raise ValueError("corrupted reply from the bridge")
        status = r[2]
        i_dis, i_chg, state, cause, soc_pct, tc, soh, psi = struct.unpack("<ffBBffff", r[3:29])
        return status, i_dis, i_chg, state, cause, soc_pct, tc, soh, psi


def find_cli():
    hits = glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe")
    return sorted(hits)[-1] if hits else None


def reset_ecu(cli):
    r = subprocess.run([cli, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True, text=True)
    if "reset is performed" not in r.stdout:
        sys.exit("ECU reset failed (ST-LINK connected? CubeIDE debug session closed?)\n" + r.stdout[-400:])
    time.sleep(1.3)                              # açılış: LED 0.9 s + CAN kurulumu; 2 s'lik açılış süresi içinde


def run(link, scenario, n_max, cli):
    inp = pd.read_csv(os.path.join(HERE, "data", f"{scenario}_in.csv"))
    ref = pd.read_csv(os.path.join(HERE, "data", f"{scenario}_c.csv"))
    n = len(inp) if n_max <= 0 else min(n_max, len(inp))
    if cli:
        reset_ecu(cli)
    out = np.zeros((n, 6)); state = np.zeros(n, int); cause = np.zeros(n, int)
    n_stale = 0
    vals = inp[["I", "V", "T", "V_min", "V_max", "T_amb"]].values
    t0 = time.time()
    for k in range(n):
        st, i_dis, i_chg, state[k], cause[k], soc_pct, tc, soh, psi = link.xfer(frame(0, vals[k]))
        if st & (ST_SER_CRC | ST_TX_ERR | ST_CAN_CRC) or (st & 3) != 3:
            raise RuntimeError(f"{scenario}: sample {k}: bridge status 0x{st:02X} "
                               f"(bit0/1 missing = no answer from the ECU, 4 serial CRC, 8 CAN tx, 0x20 CAN CRC)")
        if st & ST_STALE and k > 0:
            n_stale += 1
        out[k] = soc_pct, tc, i_dis, i_chg, psi, soh
        if (k + 1) % 2000 == 0:
            print(f"  {scenario}: {k + 1}/{n} samples ({time.time() - t0:.0f} s)")
    dt = time.time() - t0
    r = ref.iloc[:n]
    d = dict(soc_pts=np.abs(out[:, 0] - 100 * r.soc.values), tc=np.abs(out[:, 1] - r.tc.values),
             i_dis=np.abs(out[:, 2] - r.i_dis.values), i_chg=np.abs(out[:, 3] - r.i_chg.values),
             psi=np.abs(out[:, 4] - r.psi.values))
    row = dict(Scenario=scenario, Samples=n)
    for key, v in d.items():
        row[key + "_maxdiff"] = v.max()
        row[key + "_lsb"] = v.max() / LSB[key]
    row.update(State_mismatch=int(np.sum(state != r.state.values)),
               Cause_mismatch=int(np.sum(cause != (r.cause.values & 0xFF))),
               Extra_steps=n_stale, Final_state=NAMES.get(int(state[-1]), "?"),
               ms_per_sample=1000 * dt / n)
    pd.DataFrame(dict(soc_pct=out[:, 0], tc=out[:, 1], i_dis=out[:, 2], i_chg=out[:, 3], psi=out[:, 4],
                      soh_pct=out[:, 5], state=state, cause=cause)
                 ).to_csv(os.path.join(HERE, f"can_replay_{scenario}.csv"), index=False)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Arduino bridge port (e.g. COM10)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--n", type=int, default=0, help="samples per scenario (0 = all)")
    ap.add_argument("--scenarios", default="S6_ntc_open,S3_deep_weak")
    ap.add_argument("--no-reset", action="store_true", help="do not reset the ECU through ST-LINK")
    ap.add_argument("--cli", default=None, help="path to STM32_Programmer_CLI.exe")
    a = ap.parse_args()
    cli = None
    if not a.no_reset:
        cli = a.cli or find_cli()
        if not cli:
            sys.exit("STM32_Programmer_CLI.exe not found: give --cli PATH or use --no-reset")
    link = Bridge(a.port, a.baud)
    rows = [run(link, s, a.n, cli) for s in a.scenarios.split(",")]
    R = pd.DataFrame(rows)
    pd.set_option("display.width", 250)
    print("\n=== CAN replay: STM32G431 ECU on the real CAN bus vs PC SIL (same C code, same inputs) ===")
    cols = ["Scenario", "Samples"] + [k + "_lsb" for k in LSB] + ["State_mismatch", "Cause_mismatch",
                                                                   "Extra_steps", "Final_state", "ms_per_sample"]
    print(R[cols].to_string(index=False, float_format=lambda v: f"{v:.3g}"))
    print("(*_lsb: largest difference in units of the CAN resolution; ~0.5 is CAN rounding)")
    ok1 = all((R[k + "_maxdiff"] <= 0.5 * LSB[k] + ALGO[k]).all() for k in LSB)
    ok2 = (R.State_mismatch == 0).all() and (R.Cause_mismatch == 0).all()
    ok3 = (R.Extra_steps == 0).all()
    print(f"Acceptance 1 (values within 0.5 CAN LSB + PIL float margin): {'PASS' if ok1 else 'FAIL'}")
    print(f"Acceptance 2 (identical state and cause sequences):     {'PASS' if ok2 else 'FAIL'}")
    print(f"Acceptance 3 (exactly one bms_step per sample):         {'PASS' if ok3 else 'FAIL'}")
    R.to_csv(os.path.join(HERE, "can_replay_results.csv"), index=False)


if __name__ == "__main__":
    main()
