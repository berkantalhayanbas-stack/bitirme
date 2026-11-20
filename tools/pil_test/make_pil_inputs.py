"""PIL girdi senaryolarını (data/<s>_in.csv) dijital ikizin kapalı çevriminden yeniden üretir.

Girdi = ECU'nun CAN'dan göreceği ölçümler (closed_loop.run günlüğündeki im, vm, tm, vmin_m, vmax_m;
gürültü + kuantalama dahil). Kapalı çevrim: ikizin akımı ECU'nun I_allow'u ile kırpılır, yani açılış
sıfır akım kontrolü (A9, ilk 1 s kontaktör açık) ve rampa girdide gerçekçi görünür.
    python tools/pil_test/make_pil_inputs.py [--twin C:\\bitirme\\twin]
Sonra: python tools/pil_test/sil_ref.py (C referansları) ve python tools/pil_test/pil_test.py (kart).
"""
import argparse
import os
import sys

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--twin", default=r"C:\bitirme\twin")
a = ap.parse_args()
sys.path.insert(0, a.twin)
os.chdir(a.twin)
from closed_loop import run  # noqa: E402

I = pd.read_csv(os.path.join(a.twin, "..", "simulink", "twin_reference.csv")).i_pack.values
SCEN = {"S3_deep_weak": (I, dict(soc0=0.4, weak=True)),
        "S6_ntc_open": (I, dict(fault="t_open")),
        "S8_ext_heat": (I[:9000], dict(fault="heat", t_fault=600.0))}
for name, (cur, kw) in SCEN.items():
    d = run(cur, **kw)
    out = pd.DataFrame({"I": d.im, "V": d.vm, "T": d.tm, "V_min": d.vmin_m, "V_max": d.vmax_m,
                        "T_amb": kw.get("T_amb", 25.0)})
    out = out.round({"I": 1, "V": 1, "T": 1, "V_min": 3, "V_max": 3})
    p = os.path.join(HERE, "data", f"{name}_in.csv")
    out.to_csv(p, index=False)
    print(f"{name}: {len(out)} örnek, ilk 1 s ortalama akım {out.I[:10].mean():+.2f} A -> {p}")
