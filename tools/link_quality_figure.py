r"""Uzaktan izleme bağlantı kalitesi figürü (tez): gateway.py yığın CSV'sinden (build/web/link_*.csv).

    python tools/link_quality_figure.py C:\bitirme\results\link_quality_window_20261121.csv
    -> C:\bitirme\results\link_quality_20261121.png (+ .pdf)

(a) yığın başına yükleme RTT'si zamana göre (p50 / p95 çizgileri), (b) gecikmelerin birikimli
dağılımı (log eksen): RTT, uçtan uca (yığının en yeni / en eski satırı). Hepsi PC saatiyle.
"""
import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FixedLocator, NullLocator, ScalarFormatter

import thesis_style as ts

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--out", default=r"C:\bitirme\results\link_quality_20261121.png")
a = ap.parse_args()

R = list(csv.DictReader(open(a.csv)))
seq = np.array([int(r["seq"]) for r in R], float)
t = (seq - seq[0]) / 60000.0                                   # [dk]
col = lambda c: np.array([float(r[c]) for r in R])
rtt, e_new, e_old, jit = col("rtt_ms"), col("age_last_ms"), col("age_first_ms"), col("jitter_ms")
rows = int(col("rows").sum())
miss = int(col("uart_missing").sum())
p = lambda v, q: float(np.percentile(v, q, method="nearest"))

ts.apply()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.2, 2.9), gridspec_kw={"width_ratios": [1.35, 1]})

# ---- (a) RTT zaman serisi
ax1.plot(t, rtt, color=ts.SERIES[0], lw=0.8)
for q in (50, 95):                     # etiket eksenin sağında: seriyle çakışmasın
    ts.ref_line(ax1, y=p(rtt, q))
    ax1.annotate(f"p{q} {p(rtt, q):.0f}", (1.0, p(rtt, q)), xycoords=("axes fraction", "data"), xytext=(3, 0),
                 textcoords="offset points", va="center", fontsize=7.5, color=ts.INK2, annotation_clip=False)
ax1.set_yscale("log")
ax1.set_xlim(0, t[-1])
ax1.set_xlabel("Time [min]")
ax1.set_ylabel("Upload round-trip time [ms]")
ts.panel_label(ax1, f"(a) Upload RTT per 1 s batch (n = {len(R)})")

# ---- (b) birikimli dağılım
for v, name, c in ((rtt, "Upload RTT", ts.SERIES[0]),
                   (e_new, "E2E, newest row", ts.SERIES[1]),
                   (e_old, "E2E, oldest row", ts.SERIES[2])):
    x = np.sort(v)
    y = np.arange(1, len(x) + 1) / len(x)
    ax2.step(x, y, where="post", color=c, lw=1.6, label=name)
    ax2.plot([p(v, 50)], [0.5], "o", ms=4, color=c, mec=ts.SURFACE, mew=1.0, zorder=3)
ts.ref_line(ax2, y=0.95)
ax2.annotate("p95", (0.0, 0.95), xycoords=("axes fraction", "data"), xytext=(3, 2), textcoords="offset points",
             va="bottom", fontsize=7.5, color=ts.INK2)
ax2.set_xscale("log")
ax2.set_ylim(0, 1.02)
ax2.set_xlim(70, 4000)
ax2.set_xlabel("Latency [ms]")
ax2.set_ylabel("Cumulative fraction of batches")
ts.panel_label(ax2, "(b) Latency distribution")

def ms_ticks(axis, ticks):              # log eksende düz ms etiketleri
    axis.set_major_locator(FixedLocator(ticks))
    axis.set_minor_locator(NullLocator())
    axis.set_major_formatter(ScalarFormatter())
ms_ticks(ax1.yaxis, [100, 200, 500, 1000, 2000])
ms_ticks(ax2.xaxis, [100, 300, 1000, 3000])
ax1.set_ylim(80, 2000)

fig.text(0.01, 0.01, f"{t[-1]:.0f} min window, {rows:,} rows at 10 Hz, UART loss {100 * miss / (rows + miss):.1f} %, "
                     f"UART jitter p95 {p(jit, 95):.0f} ms. E2E = UART receive to server ack. Latencies on the PC clock; ~1 s of end-to-end delay is "
                     f"the 1 s batching.", fontsize=7, color=ts.INK2)
h, l = ax2.get_legend_handles_labels()        # lejant (b)'nin altında: eğrilere binmesin
fig.tight_layout(rect=(0, 0.11, 1, 1))
fig.legend(h, l, loc="lower right", bbox_to_anchor=(0.99, 0.05), ncol=3, fontsize=7.5, frameon=False)
os.makedirs(os.path.dirname(a.out), exist_ok=True)
ts.save(fig, a.out, pdf=True)
print(a.out)
