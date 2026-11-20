r"""bms/sim/mc_cycle.h'yi üretir: dijital ikizin sürüş çevrimi (simulink/twin_reference.csv, 10 Hz
paket akımı) 1 s ortalamalarla, 1 A çözünürlüklü int8 tablo (Monte Carlo'da akım talebi şablonu).
    python tools/mc/make_cycle.py [--ref C:\bitirme\simulink\twin_reference.csv]
"""
import argparse
import os

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ap = argparse.ArgumentParser()
ap.add_argument("--ref", default=r"C:\bitirme\simulink\twin_reference.csv")
a = ap.parse_args()
i = pd.read_csv(a.ref).i_pack.values
n = len(i) // 10
c = np.clip(np.round(i[:n * 10].reshape(n, 10).mean(axis=1)), -127, 127).astype(int)
rows = [", ".join(f"{v:4d}" for v in c[k:k + 16]) for k in range(0, n, 16)]
body = ",\n    ".join(rows)
txt = f"""/* mc_cycle.h -- ÜRETİLMİŞ DOSYA (tools/mc/make_cycle.py), elle düzenleme.
 * Dijital ikizin sürüş çevrimi (twin_reference.csv i_pack), 1 s ortalaması, 1 A çözünürlük [A].
 * {n} s, en düşük {c.min()} A, en yüksek {c.max()} A. Monte Carlo akım talebi = k_cur x çevrim(t + kaydırma).
 */
#ifndef MC_CYCLE_H
#define MC_CYCLE_H
#include <stdint.h>
#define MC_CYCLE_N {n}
static const int8_t MC_CYCLE[MC_CYCLE_N] = {{
    {body} }};
#endif
"""
open(os.path.join(ROOT, "bms", "sim", "mc_cycle.h"), "w", encoding="utf-8", newline="\n").write(txt)
print(f"bms/sim/mc_cycle.h: {n} samples, {c.min()}..{c.max()} A")
