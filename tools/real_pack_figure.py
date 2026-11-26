r"""Gerçek paket ölçümü ve dijital ikizle karşılaştırma (tez): aracın 19S12P LG M50 paketi, DT-830D multimetre
(26 Kasım 2026, dinlenmede). Ölçümler bu dosyadaki sabitlerdir (tek kaynak).

    python tools/real_pack_figure.py
    -> C:\bitirme\results\real_pack_cell_voltages.png (+ .pdf), real_pack_vs_twin.csv, real_pack_measurements.csv

(a) 19 grup gerilimi, +-1 hane (10 mV) hata çubuğu, sağ eksende OCV tablosundan SOC (Kirkaldy 2024 M50T GITT,
    bms_config.h SOH_OCV_* ile aynı). 3,76 / 3,78 V gruplarının sırası bilinmiyor: 1. ve 19. gruba konur.
(b) En düşük gruba göre fark, büyükten küçüğe: gerçek paket ile ikizin başlangıç saçılması (twin_model.c GRP
    başlangıç SOC sütunu, aynı ortalama SOC'de OCV'ye çevrilir).
"""
import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

import thesis_style as ts

# ---------------- ölçümler (multimetre DT-830D)
V_GRP = np.array([3.76] + [3.77] * 17 + [3.78])     # 20 V kademesi, 10 mV çözünürlük [V]
V_PACK = 71.9                                        # B- -> B+, 200 V kademesi [V]
V_TAP = [3.7, 7.5, 11.3, 15.1, 18.9, 22.7, 26.5, 30.3, 34.1, 37.9, 41.8, 45.5, 49.4, 53.2, 57.0,
         60.8, 64.6, 68.5, 72.3]                    # birikimli, B-'den, 200 V kademesi [V]
RES = 0.010                                          # 20 V kademesinin bir hanesi [V]
ACC = lambda v, res=RES: 0.005 * v + 2 * res         # DT-830D DCV doğruluğu +-(%0,5 + 2 hane)

# ---------------- OCV tablosu (bms_config.h SOH_OCV_*, twin_model.c OCV_TAB)
OCV_S = [0.0, 0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8,
         0.85, 0.9, 0.95, 0.98, 1.0]
OCV_V = [2.9414, 3.0161, 3.1444, 3.2871, 3.4047, 3.4646, 3.5055, 3.5621, 3.6086, 3.6418, 3.6729, 3.7119,
         3.7594, 3.8136, 3.8737, 3.9207, 3.9641, 4.0162, 4.0619, 4.0873, 4.112, 4.1519, 4.1845]
soc_of = lambda v: 100.0 * np.interp(v, OCV_V, OCV_S)
ocv_of = lambda s: np.interp(np.asarray(s) / 100.0, OCV_S, OCV_V)

# ---------------- ikizin başlangıç SOC saçılması [0,1 % SOC] (twin_model.c GRP[.][2])
TWIN_DS = np.array([2, -3, 5, 0, -6, 8, -10, 3, -2, 12, -4, 1, 6, -8, 0, 4, -5, 9, -1]) * 0.1
BAL_DV_ON = 0.010                                    # bms_config.h

ap = argparse.ArgumentParser()
ap.add_argument("--out", default=r"C:\bitirme\results\real_pack_cell_voltages.png")
a = ap.parse_args()
out_dir = os.path.dirname(a.out)

soc = soc_of(V_GRP)
soc_mean = soc_of(V_GRP.mean())
dv_real = np.sort(V_GRP - V_GRP.min())[::-1] * 1000
v_twin = ocv_of(soc_mean + TWIN_DS)
dv_twin = np.sort(v_twin - v_twin.min())[::-1] * 1000
slope = (ocv_of(soc_mean + 0.5) - ocv_of(soc_mean - 0.5)) * 1000   # mV / % SOC

# ---------------- figür
ts.apply()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 3.0), gridspec_kw={"width_ratios": [1.5, 1]})
g = np.arange(1, 20)
ax1.errorbar(g, V_GRP, yerr=RES, fmt="o", ms=4, color=ts.BLUE, ecolor=ts.BLUE, elinewidth=0.8, capsize=2,
             label="Measured (±1 digit)")
ts.ref_line(ax1, y=V_GRP.min() + BAL_DV_ON)
ax1.plot([], [], ls="--", lw=0.8, color=ts.MUTED, label="Balancing threshold (min + 10 mV)")
ax1.set_xlabel("Series group (positions of the 3.76 / 3.78 V groups assumed)")
ax1.set_ylabel("Rest voltage [V]")
ax1.set_xticks([1, 5, 10, 15, 19])
ax1.set_xlim(0.3, 19.7)
ax1.set_ylim(3.735, 3.805)
sec = ax1.secondary_yaxis("right", functions=(soc_of, ocv_of))
sec.set_ylabel("SOC from OCV table [%]")
ax1.legend(loc="upper left", frameon=False)
ts.panel_label(ax1, "(a) Real pack, 19 groups")

r = np.arange(1, 20)
err = np.full(19, RES * 1000)
err[-1] = 0.0                                        # en düşük grup tanım gereği 0
ax2.errorbar(r, dv_real, yerr=err, fmt="o", ms=4, color=ts.BLUE, ecolor=ts.BLUE, elinewidth=0.8,
             capsize=2, label=f"Real pack (max {dv_real.max():.0f} mV)")
ax2.plot(r + 0.3, dv_twin, "s", ms=3.5, mfc="none", color=ts.MUTED, label=f"Twin initial state (max {dv_twin.max():.0f} mV)")
ts.ref_line(ax2, y=BAL_DV_ON * 1000)
ax2.set_xlabel("Rank (largest first)")
ax2.set_ylabel("Voltage above lowest group [mV]")
ax2.set_xticks([1, 5, 10, 15, 19])
ax2.set_ylim(-3, 34)
ax2.legend(loc="upper right", frameon=False)
ts.panel_label(ax2, "(b) Spread: real vs twin")
fig.tight_layout()
ts.save(fig, a.out, pdf=True)

# ---------------- ölçüm tablosu
with open(os.path.join(out_dir, "real_pack_measurements.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["group", "v_group_V", "soc_ocv_pct", "v_tap_V", "v_tap_minus_sum_V"])
    cum = np.cumsum(V_GRP)
    for i in range(19):
        w.writerow([i + 1, f"{V_GRP[i]:.2f}", f"{soc[i]:.2f}", f"{V_TAP[i]:.1f}", f"{V_TAP[i] - cum[i]:+.2f}"])

# ---------------- gerçek paket - ikiz tablosu
CAP = 12 * 4.85
rows = [
    ("Cell", "LG INR21700 M50T (4.85 Ah min)", "LG INR21700 M50 family (label LGG8M5021700, no 'T')", "yes (same rating)"),
    ("Configuration", "19S12P (228 cells)", "19S12P (count + 19 balance leads)", "yes"),
    ("Group capacity", f"{CAP:.1f} Ah (12 x 4.85 Ah)", "58.2 Ah min, ~60 Ah typical", "yes (twin conservative)"),
    ("Discharge limit", "87.3 A (1.5C)", "datasheet 12 x 7.275 A = 87.3 A", "yes"),
    ("Charge limit", "40.7 A (0.7C); 17.46 A below 10 C", "datasheet, same", "yes"),
    ("Voltage window", "OV 4.25 V / UV 2.5 V per cell", "full 79.8 V, cut-off 47.5 V (19S)", "yes"),
    ("Motor demand", "aggressive drive cycle", "3000 W / ~72 V = ~42 A (0.72C) rated", "within limits"),
    ("Pack voltage at rest", "-", f"{V_PACK:.1f} V direct (+-{ACC(V_PACK, 0.1):.2f} V); "
                                  f"sum of groups {V_GRP.sum():.2f} V", "consistent"),
    ("Group voltage spread", f"initial SOC spread -1.0..+1.2 % = {dv_twin.max():.0f} mV",
     f"{dv_real.max():.0f} mV (3.76..3.78 V; true 10-30 mV)", "yes (same magnitude)"),
    ("SOC from OCV", "-", f"{soc.min():.1f} / {soc[1]:.1f} / {soc.max():.1f} % (mean {soc_mean:.1f} %)", "-"),
    ("OCV slope at this SOC", "-", f"{slope:.1f} mV per % SOC", "-"),
    ("Meter accuracy (20 V range)", "-", f"+-{ACC(3.77) * 1000:.0f} mV absolute, 10 mV resolution", "limits relative comparison"),
]
with open(os.path.join(out_dir, "real_pack_vs_twin.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["quantity", "twin_firmware", "real_pack", "match"])
    w.writerows(rows)

print(f"SOC {soc.min():.2f} / {soc[1]:.2f} / {soc.max():.2f} %, mean {soc_mean:.2f} %, slope {slope:.2f} mV/%")
print(f"spread real {dv_real.max():.0f} mV, twin {dv_twin.max():.1f} mV; pack {V_PACK} V vs sum {V_GRP.sum():.2f} V")
print("->", a.out)
