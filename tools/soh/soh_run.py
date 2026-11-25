"""Capacity SOH: long-term SIL run, C/Python equivalence and thesis figure (docs/soh_design.md).

    python tools/soh/soh_run.py [--days 100] [--soh-end 0.8] [--no-fig]

1. Builds tools/soh/soh_sil.c with bms/common + bms/sim/twin_model.c (PATH gcc or MATLAB MinGW).
2. Runs the closed loop (twin + bms_step + soh_step) with capacity feedback to the EKF and without it.
3. Replays the first 3 simulated days of the C trace through C:\\bitirme\\twin\\soh_capacity.py
   and requires identical events and soh_c / soh_disp (float32 in both).
4. Figure: C:\\bitirme\\results\\soh_learning.png (+ .pdf): true / learned / displayed SOH and per-measurement
   ratios; EKF SOC error with and without feedback.
Prints a summary; intermediate CSVs go to build/soh/.
"""
import argparse
import csv
import glob
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, "build", "soh")
TWIN = r"C:\bitirme\twin"
RESULTS = r"C:\bitirme\results"


def gcc():
    g = shutil.which("gcc") or next(iter(glob.glob(
        r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
    if not g:
        sys.exit("gcc not found")
    return g


def build():
    g = gcc()
    exe = os.path.join(OUT, "soh_sil.exe")
    com, sim = os.path.join(ROOT, "bms", "common"), os.path.join(ROOT, "bms", "sim")
    srcs = [os.path.join(HERE, "soh_sil.c"), os.path.join(com, "bms_algo.c"), os.path.join(com, "soh_tree.c"),
            os.path.join(sim, "twin_model.c")]
    env = dict(os.environ, PATH=os.path.dirname(g) + os.pathsep + os.environ["PATH"])
    subprocess.run([g, "-O2", "-std=c99", "-Wall", "-I", com, "-I", sim, *srcs, "-lm", "-o", exe],
                   check=True, env=env)
    return exe, env


def read(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def equivalence(trace_path):
    sys.path.insert(0, TWIN)
    import numpy as np
    import soh_capacity as sc
    rows = read(trace_path)
    s, rec, bad = None, None, 0
    for k, r in enumerate(rows):
        if r["boot"] == "1":
            s = sc.SohCapacity(rec)
        ev = s.step(float(r["I"]), float(r["V"]), float(r["T"]), float(r["k_r"]),
                    int(r["t_valid"]), int(r["i_valid"]), int(r["can_ok"]))
        if s.persist:
            rec = dict(s.r)
            s.persist = 0
            s.ah_saved, s.kr_saved = rec["ah_total"], rec["k_r"]
        if (ev != int(r["event"]) or np.float32(r["soh_c"]) != s.r["soh_c"]
                or np.float32(r["soh_disp"]) != s.r["soh_disp"]):
            bad += 1
            if bad <= 5:
                print(f"  mismatch at step {k}: C ev {r['event']} soh_c {r['soh_c']} | "
                      f"Py ev {ev} soh_c {s.r['soh_c']}")
    n_ev = sum(1 for r in rows if r["event"] != "0")
    return len(rows), n_ev, bad


def figure(days, ev, days0, png):
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import thesis_style as ts
    import matplotlib.pyplot as plt
    ts.apply()
    d = [int(r["day"]) for r in days]
    acc = [r for r in ev if r["event"] == "2"]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True, gridspec_kw=dict(height_ratios=[2.2, 1]))
    a1.plot(d, [100 * float(r["soh_true"]) for r in days], color=ts.INK, lw=1.6, label="True capacity (twin)")
    a1.scatter([float(r["day"]) for r in acc],
               [100 * float(r["ratio"]) for r in acc], s=6, color=ts.MUTED, label="Single measurement (rest to rest)")
    a1.plot(d, [100 * float(r["soh_c"]) for r in days], color=ts.BLUE, lw=1.4, label="Learned SOH (stored in flash)")
    a1.plot(d, [100 * float(r["soh_disp"]) for r in days], color=ts.ORANGE, lw=1.0, ls="--",
            label="Displayed SOH")
    a1.set_ylabel("Maximum capacity [%]")
    a1.legend(loc="lower left", frameon=False)
    a2.plot(d, [100 * float(r["soc_err_max"]) for r in days0], color=ts.MUTED, lw=1.2,
            label="EKF with nominal capacity")
    a2.plot(d, [100 * float(r["soc_err_max"]) for r in days], color=ts.BLUE, lw=1.4,
            label="EKF with learned capacity")
    a2.set_ylabel("Max |SOC error| [%]")
    a2.set_xlabel("Simulated day (1 drive + 1 charge per day, ECU reboots daily)")
    a2.legend(loc="upper left", frameon=False)
    ts.save(fig, png, pdf=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", type=int, default=100)
    ap.add_argument("--soh-end", type=float, default=0.8)
    ap.add_argument("--no-fig", action="store_true")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    exe, env = build()
    p = lambda n: os.path.join(OUT, n)
    subprocess.run([exe, str(a.days), str(a.soh_end), "1", p("events.csv"), p("days.csv"), p("trace.csv"), "3"],
                   check=True, env=env, stdout=subprocess.DEVNULL)
    subprocess.run([exe, str(a.days), str(a.soh_end), "0", p("events_nofb.csv"), p("days_nofb.csv")],
                   check=True, env=env, stdout=subprocess.DEVNULL)
    days, days0, ev = read(p("days.csv")), read(p("days_nofb.csv")), read(p("events.csv"))
    acc = [r for r in ev if r["event"] == "2"]
    dr = [float(r["ratio"]) - float(r["soh_true"]) for r in acc]
    bias = sum(dr) / len(dr)
    sd = (sum((x - bias) ** 2 for x in dr) / len(dr)) ** 0.5
    err = [float(r["soh_c"]) - float(r["soh_true"]) for r in days[3:]]
    print(f"days {a.days}, true SOH 1.00 -> {a.soh_end:.2f}, measurements accepted {len(acc)}, "
          f"rejected {sum(1 for r in ev if r['event'] == '4')}")
    print(f"single measurement - true: bias {100 * bias:+.2f} %, sd {100 * sd:.2f} %")
    print(f"learned SOH - true (day >= 3): mean {100 * sum(err) / len(err):+.2f} %, "
          f"max |.| {100 * max(abs(x) for x in err):.2f} %; final {100 * float(days[-1]['soh_c']):.1f} % "
          f"vs {100 * float(days[-1]['soh_true']):.1f} %")
    print(f"EKF max |SOC error| last day: with feedback {100 * float(days[-1]['soc_err_max']):.2f} %, "
          f"without {100 * float(days0[-1]['soc_err_max']):.2f} %")
    n, n_ev, bad = equivalence(p("trace.csv"))
    print(f"C/Python equivalence: {n} steps, {n_ev} events, mismatches {bad} -> {'PASS' if bad == 0 else 'FAIL'}")
    if not a.no_fig:
        png = os.path.join(RESULTS, "soh_learning.png")
        figure(days, ev, days0, png)
        print("figure:", png)
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
