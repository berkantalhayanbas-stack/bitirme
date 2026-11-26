"""PC SIL referanslarını yeniden üretir: sil_ref.c + bms/common -> build/sil_ref.exe -> data/<s>_c.csv.
bms/common (OCV, karar, gözlemci...) değişince çalıştırılır; sonra pil_test.py kartı buna karşı sınar.
    python tools/pil_test/sil_ref.py [--check]   (--check: yazmadan mevcut _c.csv ile karşılaştır)
Derleyici: PATH'teki gcc, yoksa MATLAB MinGW-w64 destek paketi.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
ap = argparse.ArgumentParser()
ap.add_argument("--check", action="store_true")
a = ap.parse_args()
gcc = shutil.which("gcc") or next(iter(glob.glob(
    r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
if not gcc:
    sys.exit("gcc bulunamadi")
exe = os.path.join(ROOT, "build", "sil_ref.exe")
os.makedirs(os.path.dirname(exe), exist_ok=True)
com = os.path.join(ROOT, "bms", "common")
srcs = [os.path.join(HERE, "sil_ref.c")] + [os.path.join(com, f) for f in ("bms_algo.c", "soh_tree.c", "stress_model.c")]
env = dict(os.environ, PATH=os.path.dirname(gcc) + os.pathsep + os.environ["PATH"])
subprocess.run([gcc, "-O2", "-std=c99", "-I", com, *srcs, "-lm", "-o", exe], check=True, env=env)
for fin in sorted(glob.glob(os.path.join(HERE, "data", "*_in.csv"))):
    s = os.path.basename(fin)[:-7]
    ref = os.path.join(HERE, "data", f"{s}_c.csv")
    out = exe.replace(".exe", f"_{s}.csv") if a.check else ref
    subprocess.run([exe, fin, out], check=True, env=env)
    if a.check:
        n, o = pd.read_csv(out), pd.read_csv(ref)
        print(f"{s}: SOC max fark {100 * np.max(np.abs(n.soc - o.soc)):.5f} puan, durum farki "
              f"{int(np.sum(n.state != o.state))}, neden farki {int(np.sum(n.cause != o.cause))}, "
              f"I_dis max fark {np.max(np.abs(n.i_dis - o.i_dis)):.4f} A")
    else:
        print(f"yazildi: {os.path.relpath(ref, ROOT)} ({len(pd.read_csv(ref))} satir)")
