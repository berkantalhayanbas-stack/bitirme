r"""Pasif dengeleme kart testi figürü (tez): tools/twin_emu_run.py --bal 0.3 --cmd "1:m 3" --cmd "1:b 500"
çıktısı (R satırları). İkiz bekleme modunda (0 A); dengelemenin SOC etkisi x500 hızlandırılmış (gerilim düşümü
gerçek akımla), bu yüzden süre ekseni hem kart süresi hem eşdeğer gerçek süre olarak verilir.

    python tools/balancing_figure.py C:\bitirme\results\balancing_bench.csv
    -> C:\bitirme\results\balancing_bench.png (+ .pdf)
"""
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np

import thesis_style as ts

ON_DELAY_S = 60.0     # BAL açma beklemesi (bms_config.h)

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--out", default=r"C:\bitirme\results\balancing_bench.png")
a = ap.parse_args()

R = [r for r in csv.DictReader(open(a.csv)) if r["R"] == "R"] or list(csv.DictReader(open(a.csv)))
col = lambda c: np.array([float(r[c]) for r in R])
t = col("t") - float(R[0]["t"])
dv, dv_ecu, dsoc = col("dv_mV"), col("dv_ecu_mV"), col("dsoc")
n_on = np.array([bin(int(r["bal_mask"], 16)).count("1") for r in R])
k = int(float(R[-1]["bal_k"]))

ts.apply()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7.2, 5.0), sharex=True,
                                    gridspec_kw={"height_ratios": [1.3, 1, 0.7]})
x = t / 60
ax1.plot(x, dv, color=ts.SERIES[0], lw=1.4, label="Twin (true)")
ax1.plot(x, dv_ecu, color=ts.SERIES[1], lw=1.0, label="ECU (from CAN, 1 mV)")
ax1.set_ylabel("Group spread\nV_max - V_min [mV]")
ax1.legend(loc="upper right", fontsize=7.5)
ts.panel_label(ax1, f"(a) Voltage spread across 19 groups: {dv[0]:.1f} -> {dv[-1]:.1f} mV")

ax2.plot(x, dsoc, color=ts.SERIES[0], lw=1.4)
ax2.set_ylabel("Group SOC\nspread [%]")
ax2.set_ylim(0, None)
ts.panel_label(ax2, f"(b) SOC spread (twin): {dsoc[0]:.1f} -> {dsoc[-1]:.1f} %")

ax3.step(x, n_on, where="post", color=ts.SERIES[2], lw=1.2)
ax3.set_ylabel("Bleed resistors\non [-]")
ax3.set_ylim(0, 19.5)
ax3.set_yticks([0, 5, 10, 15, 19])
ax3.set_xlabel(f"Bench time [min]  (SOC effect accelerated x{k}: {t[-1] / 60:.0f} min ~ {t[-1] * k / 3600:.0f} h real time)")
on = np.where(n_on > 0)[0]
t_bal = (t[on[-1] + 1] if on[-1] + 1 < len(t) else t[-1]) - t[on[0]] if len(on) else 0.0
ts.panel_label(ax3, f"(c) Bleed resistors on (ECU 0x210): active {t_bal:.0f} s bench ~ {t_bal * k / 3600:.1f} h real")
ax1.annotate("residual below the 10 mV start threshold", (x[-1], dv[-1]), xytext=(0, 6), textcoords="offset points",
             ha="right", va="bottom", fontsize=7.5, color=ts.INK2)
for ax in (ax1, ax2, ax3):
    ts.ref_line(ax, x=ON_DELAY_S / 60)
ax1.annotate("60 s on-delay", (ON_DELAY_S / 60, 1.0), xycoords=("data", "axes fraction"), xytext=(3, -3),
             textcoords="offset points", ha="left", va="top", fontsize=7.5, color=ts.INK2)
ax1.set_xlim(0, x[-1])

fig.tight_layout()
ts.save(fig, a.out, pdf=True)
print(a.out)
