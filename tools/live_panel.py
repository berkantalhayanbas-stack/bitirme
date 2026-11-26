"""
Adım 9: canlı izleme paneli. ECU (APP_ECU, -DBMS_TELEMETRY=2) her adımda UART'a şu satırı yazar:
  DX,ms,state,cause,soc_pct,tc,ts,i_allow_dis,i_allow_chg,psi,soh_pct,I,V,T,V_min,V_max
Panel bu satırları okur, CSV'ye kaydeder ve canlı çizer: SOC, sıcaklıklar (Tc tahmini, Ts tahmini,
ölçülen T), akım (ölçülen I, +I_allow_dis, -I_allow_chg), hücre gerilimleri (V_min / V_max; adım 8'de
19 grup), durum (NORMAL / DERATE / LIMP / SAFE). Diğer satırlar (1 s'lik rapor, komut cevapları)
konsola yazılır.

Kullanım:
  python tools/fw_build.py --app 4 --define BMS_TELEMETRY=2 --flash
  python tools/live_panel.py --port COM9                       # canlı pencere + build/live/<zaman>.csv
  python tools/live_panel.py --port COM9 --duration 60 --png p.png --no-gui   # pencere yok, sonda PNG
"""
import argparse
import csv
import os
import threading
import time
from collections import deque

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLS = ["ms", "state", "cause", "soc_pct", "tc", "ts", "i_allow_dis", "i_allow_chg", "psi", "soh_pct",
        "I", "V", "T", "V_min", "V_max"]
STATES = {1: "NORMAL", 2: "DERATE", 3: "LIMP", 4: "SAFE"}


class Reader(threading.Thread):
    def __init__(self, port, csv_path, window_s):
        super().__init__(daemon=True)
        import serial
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.n = int(window_s * 10) + 10
        self.data = {c: deque(maxlen=self.n) for c in COLS + ["t_pc"]}
        self.lock = threading.Lock()
        self.count = 0
        self.stop = False
        os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
        self.f = open(csv_path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_pc"] + COLS)
        self.t0 = time.time()

    def run(self):
        buf = b""
        while not self.stop:
            buf += self.s.read(2048)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode(errors="replace").strip()
                if line.startswith("DX,"):
                    p = line[3:].split(",")
                    if len(p) < len(COLS):          # yeni alanlar sonda (24/30/36 alanlı satır): ilk alanlar yeter
                        continue
                    p = p[:len(COLS)]
                    try:
                        v = [float(x) for x in p]
                    except ValueError:
                        continue
                    t = time.time() - self.t0
                    self.w.writerow([f"{t:.3f}"] + p)
                    with self.lock:
                        self.data["t_pc"].append(t)
                        for c, x in zip(COLS, v):
                            self.data[c].append(x)
                        self.count += 1
                elif line:
                    print(line)
        self.f.close()

    def snapshot(self):
        with self.lock:
            return {k: list(v) for k, v in self.data.items()}


def build_figure():
    import matplotlib.pyplot as plt
    import thesis_style as ts                       # ortak tez figür stili
    ts.apply()
    fig, ax = plt.subplots(5, 1, figsize=(11, 10), sharex=True,
                           gridspec_kw=dict(height_ratios=[1, 1, 1.2, 1, 0.6]))
    L = {}
    L["soc"], = ax[0].plot([], [], "b-", lw=1.4, label="SOC estimate (EKF)")
    ax[0].set_ylabel("SOC [%]")
    L["tc"], = ax[1].plot([], [], "r-", lw=1.3, label="T_core estimate")
    L["ts"], = ax[1].plot([], [], "m--", lw=1.0, label="T_surf estimate")
    L["T"], = ax[1].plot([], [], ".", color=ts.MUTED, ms=2, label="T measured")
    ax[1].set_ylabel("Temperature [°C]")
    L["I"], = ax[2].plot([], [], "k-", lw=0.9, label="I measured")
    L["idis"], = ax[2].plot([], [], "g-", lw=1.4, label="+I_allow discharge")
    L["ichg"], = ax[2].plot([], [], "c-", lw=1.4, label="−I_allow charge")
    ax[2].set_ylabel("Current [A]")
    L["vmin"], = ax[3].plot([], [], "b-", lw=1.1, label="V_cell min")
    L["vmax"], = ax[3].plot([], [], "r-", lw=1.1, label="V_cell max")
    ax[3].set_ylabel("Cell voltage [V]")
    L["state"], = ax[4].step([], [], "k-", where="post", lw=1.5)
    ax[4].set_yticks([1, 2, 3, 4]); ax[4].set_yticklabels([STATES[k] for k in (1, 2, 3, 4)])
    ax[4].set_ylim(0.5, 4.5); ax[4].set_xlabel("Time [s]")
    for a in ax[:4]:
        a.grid(alpha=0.3)
        a.legend(loc="upper left", fontsize=8, ncol=3)
    ax[4].grid(alpha=0.3)
    title = fig.suptitle("BMS ECU live monitor (STM32G431, CAN 0x101/0x102 -> bms_step)")
    fig.tight_layout()
    return fig, ax, L, title


def update(fig, ax, L, title, d, csv_path):
    if not d["ms"]:
        return
    t = [x / 1000.0 for x in d["ms"]]
    L["soc"].set_data(t, d["soc_pct"])
    L["tc"].set_data(t, d["tc"]); L["ts"].set_data(t, d["ts"]); L["T"].set_data(t, d["T"])
    L["I"].set_data(t, d["I"]); L["idis"].set_data(t, d["i_allow_dis"])
    L["ichg"].set_data(t, [-x for x in d["i_allow_chg"]])
    L["vmin"].set_data(t, d["V_min"]); L["vmax"].set_data(t, d["V_max"])
    L["state"].set_data(t, d["state"])
    for a in ax[:4]:
        a.relim(); a.autoscale_view()
    ax[4].set_xlim(t[0], max(t[-1], t[0] + 1))
    st = STATES.get(int(d["state"][-1]), "?")
    title.set_text(f"BMS ECU live monitor  |  {st}  cause {int(d['cause'][-1])}  |  SOC {d['soc_pct'][-1]:.2f} %  "
                   f"T_core {d['tc'][-1]:.1f} °C  I_allow {d['i_allow_dis'][-1]:.1f}/{d['i_allow_chg'][-1]:.1f} A  "
                   f"SOH {d['soh_pct'][-1]:.1f} %  |  log: {os.path.basename(csv_path)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--window", type=float, default=120.0, help="seconds shown")
    ap.add_argument("--csv", default=None, help="CSV path (default build/live/<time>.csv)")
    ap.add_argument("--duration", type=float, default=0, help="stop after N s (0 = until window closed)")
    ap.add_argument("--png", default=None, help="save the final figure")
    ap.add_argument("--no-gui", action="store_true")
    a = ap.parse_args()
    csv_path = a.csv or os.path.join(ROOT, "build", "live", time.strftime("live_%Y%m%d_%H%M%S.csv"))
    r = Reader(a.port, csv_path, a.window)
    r.start()
    import matplotlib
    if a.no_gui:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax, L, title = build_figure()
    t_start = time.time()
    if a.no_gui:
        while time.time() - t_start < (a.duration or 30):
            time.sleep(0.5)
    else:
        plt.ion()
        plt.show(block=False)
        while plt.fignum_exists(fig.number) and (not a.duration or time.time() - t_start < a.duration):
            update(fig, ax, L, title, r.snapshot(), csv_path)
            fig.canvas.draw_idle()
            plt.pause(0.25)                           # ~4 Hz yenileme
    update(fig, ax, L, title, r.snapshot(), csv_path)
    r.stop = True
    time.sleep(0.3)
    rate = r.count / max(time.time() - t_start, 1e-6)
    print(f"RESULT | samples {r.count} | rate {rate:.2f} Hz | csv {csv_path}")
    if a.png:
        fig.savefig(a.png, dpi=110)
        print(f"figure: {a.png}")


if __name__ == "__main__":
    main()
