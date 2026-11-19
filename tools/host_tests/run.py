"""bms/common PC birim testlerini derler (ASan + UBSan) ve çalıştırır.
    python tools/host_tests/run.py
Donanım gerekmez; PATH'te gcc yeter (yoksa sil_ref.py gibi MATLAB MinGW-w64'e bakar).
Çıktı: build/host_test_common(.exe). Başarısızlıkta çıkış kodu != 0.
"""
import glob
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
gcc = shutil.which("gcc") or next(iter(glob.glob(
    r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
if not gcc:
    sys.exit("gcc bulunamadi")
com = os.path.join(ROOT, "bms", "common")
exe = os.path.join(ROOT, "build", "host_test_common" + (".exe" if os.name == "nt" else ""))
os.makedirs(os.path.dirname(exe), exist_ok=True)
srcs = [os.path.join(HERE, "test_common.c")] + [os.path.join(com, f) for f in
        ("bms_algo.c", "bms_bal.c", "bms_can.c", "soh_tree.c")]
san = [] if os.name == "nt" else ["-fsanitize=address,undefined", "-fno-sanitize-recover=undefined"]
env = dict(os.environ, PATH=os.path.dirname(gcc) + os.pathsep + os.environ["PATH"])
subprocess.run([gcc, "-O1", "-g", "-std=c99", "-Wall", "-Wextra", "-Wdouble-promotion", *san,
                "-I", com, *srcs, "-lm", "-o", exe], check=True, env=env)
sys.exit(subprocess.run([exe], env=env).returncode)
