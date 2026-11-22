r"""Direnç tabanlı SOH kart testi figürü (tez): ikiz SOH'u 100/90/80/70 ayarlanır (twin_emu `y`), ECU yalnız
CAN ölçümlerinden EKF k_R'yi kestirir; SOH_R = 1 - (k_R - 1) / 1,31 (Kirkaldy 40 hücre, veri/age_resistance.py).

    python tools/soh_bench_figure.py C:\bitirme\results\soh_bench.csv
    -> C:\bitirme\results\soh_bench.png (+ .pdf), soh_bench_summary.csv

Girdi sütunları: soh_set,t_s,state,cause,soc,tc,i_meas,k_r (t_s = ECU sıfırlamasından beri).
k_R ilk 120 s donuk (EKF_N_KR_FREEZE); oturmuş değer = son 120 s'nin ortalaması.
"""
import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

import thesis_style as ts

BETA = 1.31          # k_R = 1 + BETA (1 - SOH)
FREEZE_S = 120.0
WIN_S = 120.0        # oturmuş değer penceresi (son WIN_S saniye)

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--out", default=r"C:\bitirme\results\soh_bench.png")
a = ap.parse_args()

R = list(csv.DictReader(open(a.csv)))
levels = sorted({int(r["soh_set"]) for r in R}, reverse=True)
soh_r = lambda k: 100.0 * (1.0 - (k - 1.0) / BETA)

ts.apply()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 3.0), gridspec_kw={"width_ratios": [1.4, 1]})
rows = []
for i, lv in enumerate(levels):
    d = [r for r in R if int(r["soh_set"]) == lv]
    t = np.array([float(r["t_s"]) for r in d])
    k = np.array([float(r["k_r"]) for r in d])
    st = np.array([int(r["state"]) for r in d])
    c = ts.SERIES[i]
    ax1.plot(t / 60, k, color=c, lw=1.4, label=f"{lv} %")
    k_true = 1 + BETA * (1 - lv / 100)
    ax1.axhline(k_true, color=c, lw=0.8, ls=":", zorder=1)
    m = t >= t[-1] - WIN_S
    k_end = float(np.mean(k[m]))
    est = soh_r(k_end)
    # oturma süresi: donmadan sonra k_R son değerin %2'si içine girip orada kaldığı ilk an
    inside = np.abs(k - k_end) <= 0.02 * k_end
    after = np.where((t >= FREEZE_S) & ~inside)[0]
    t_settle = float(t[after[-1] + 1]) if len(after) and after[-1] + 1 < len(t) else FREEZE_S
    rows.append(dict(soh_set=lv, k_r_true=round(k_true, 3), k_r_est=round(k_end, 3),
                     k_r_err_pct=round(100 * (k_end / k_true - 1), 2), soh_est=round(est, 1),
                     soh_err_pts=round(est - lv, 1), settle_s=round(t_settle, 0),
                     states=" ".join(ts.STATE_NAME[s] for s in sorted(set(st[t >= FREEZE_S])))))
    ax2.plot([lv], [est], "o", ms=7, color=c, mec=ts.SURFACE, mew=1.2, zorder=3)
    ts.end_label(ax2, lv, est, f"{est - lv:+.1f}", dy=0)

ts.ref_line(ax1, x=FREEZE_S / 60)
ax1.annotate("k_R frozen", (FREEZE_S / 60, 0.0), xycoords=("data", "axes fraction"), xytext=(-3, 3),
             textcoords="offset points", ha="right", va="bottom", fontsize=7.5, color=ts.INK2)
ax1.set_xlabel("Time since ECU reset [min]")
ax1.set_ylabel("EKF resistance scale k_R [-]")
ax1.legend(loc="upper left", fontsize=7.5, ncol=1, handlelength=1.2, title="Twin SOH", title_fontsize=7.5)
ts.panel_label(ax1, "(a) ECU k_R estimate (dotted: twin truth)")

lo, hi = min(min(levels), min(r["soh_est"] for r in rows)) - 4, 102
ax2.plot([lo, hi], [lo, hi], color=ts.MUTED, lw=0.8, ls="--", zorder=1)
ax2.set_xlim(lo, hi); ax2.set_ylim(lo, hi)
ax2.set_xlabel("Twin SOH (set) [%]")
ax2.set_ylabel("ECU SOH from k_R [%]")
ts.panel_label(ax2, "(b) Estimated vs set SOH")

fig.tight_layout()
ts.save(fig, a.out, pdf=True)
with open(os.path.splitext(a.out)[0] + "_summary.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)
for r in rows:
    print(r)
print(a.out)
