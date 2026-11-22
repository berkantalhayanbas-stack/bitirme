r"""Kart senaryo figürü (tez stili): tools/demo_run.py çıktısı (build/demo_<ad>.csv) + konsol kaydı (bölüm zamanları).

    python tools/demo_run.py tools/demo/modlar.txt > build/demo_modlar_final.log
    python tools/scenario_figure.py build/demo_modlar.csv build/demo_modlar_final.log C:\bitirme\results\scenario_modes.png

(a) paket akımı ve ECU'nun izin verdiği deşarj akımı, (b) ikizin gerçek çekirdek sıcaklığı / ölçülen yüzey /
ECU kestirimi, (c) ECU durumu (yalnız durum renkleri). Bölümler numaralı dikey çizgi; adları altta (İngilizce).
"""
import argparse
import csv
import os
import re

import matplotlib.pyplot as plt
import numpy as np

import thesis_style as ts

# senaryo dosyalarındaki (Türkçe) bölüm adları -> figür adı
NAMES = {
    "NORMAL": "Service -> NORMAL", "DERATE": "Fast charge -> DERATE", "LIMP-HOME": "Temp. sensor open -> LIMP",
    "STOP": "Measurement line lost -> SAFE", "WATCHDOG": "Software hang -> SAFE",
    "Hazirlik": "Preparation", "DERATE uzun basis: sogutma arizasi": "Cooling fault (hidden)",
    "STOP uzun basis: asiri isinma": "Overheating (hidden heat)", "LIMP: sensor koptu": "Temp. sensor open",
    "STOP: iletisim koptu": "Measurement line lost", "WATCHDOG: yazilim kilitlendi": "Software hang (watchdog)",
    "A2 Asiri isinma": "Overheating (hidden heat)",
}

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("log")
ap.add_argument("out")
ap.add_argument("--title", default=None)
a = ap.parse_args()

R = list(csv.DictReader(open(a.csv)))
col = lambda c: np.array([float(r[c]) for r in R])
t = col("t") / 60.0
st = col("state").astype(int)
sec = []
for line in open(a.log, encoding="utf-8", errors="replace"):
    m = re.match(r"\[\s*([\d.]+) s\] ===== (.+) =====", line.strip())
    if m:
        sec.append((float(m.group(1)) / 60.0, NAMES.get(m.group(2), m.group(2))))

ts.apply()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7.2, 5.2), sharex=True,
                                    gridspec_kw={"height_ratios": [1.1, 1.1, 0.45]})
ax1.plot(t, col("Idis"), color=ts.SERIES[1], lw=1.4, label="I_allow (ECU)")
ax1.plot(t, col("I"), color=ts.SERIES[0], lw=0.9, label="Pack current (twin)")
ax1.set_ylabel("Current [A]")
ax1.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2, fontsize=7.5, frameon=False, borderaxespad=0.1)
ts.panel_label(ax1, "(a) Current")

ax2.plot(t, col("Tc"), color=ts.SERIES[0], lw=1.4, label="Core (twin)")
ax2.plot(t, col("Ts"), color=ts.MUTED, lw=1.0, label="Surface (twin)")
ax2.plot(t, col("tc_ecu"), color=ts.SERIES[1], lw=1.0, ls="--", label="Core estimate (ECU)")
ax2.set_ylabel("Temperature [°C]")
ax2.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=3, fontsize=7.5, frameon=False, borderaxespad=0.1)
ts.panel_label(ax2, "(b) Temperature")

ts.state_band(ax3, t, st, alpha=0.9, legend=True)
ax3.set_yticks([])
ax3.set_ylim(0, 1)
ax3.legend(loc="upper center", bbox_to_anchor=(0.5, -0.9), ncol=4, fontsize=7.5, frameon=False)
ts.panel_label(ax3, "(c) ECU state (blank = no ECU message)")
ax3.set_xlabel("Bench time [min]")
ax3.set_xlim(t[0], t[-1])

for i, (ts_min, name) in enumerate(sec, 1):
    for ax in (ax1, ax2, ax3):
        ax.axvline(ts_min, color=ts.INK2, lw=0.7, ls=":", zorder=1.5)
    ax1.annotate(str(i), (ts_min, 1.0), xycoords=("data", "axes fraction"), xytext=(2, -2),
                 textcoords="offset points", ha="left", va="top", fontsize=8, color=ts.INK)
legend_txt = "   ".join(f"{i} {n}" for i, (_, n) in enumerate(sec, 1))
fig.text(0.01, 0.005, legend_txt, fontsize=7, color=ts.INK2, wrap=True)
if a.title:
    fig.suptitle(a.title, x=0.01, ha="left", fontsize=10)
fig.tight_layout(rect=(0, 0.06, 1, 1))
os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
ts.save(fig, a.out, pdf=True)
print(a.out, len(sec), "sections")
