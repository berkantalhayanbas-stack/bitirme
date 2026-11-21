r"""Kart üstü Monte Carlo (APP_MC) + aynı kodun PC (SIL) sürümü: koştur, karşılaştır, tablo/figür.

    python tools/mc_run.py --n 1000 --seed 1            # PC SIL + kart (derle, yükle, COM9'dan oku)
    python tools/mc_run.py --n 1000 --pc-only           # yalnız PC
    python tools/mc_run.py --n 1000 --stm-log build\mc_stm.txt   # kayıtlı kart çıktısıyla karşılaştır
Çıktılar (İngilizce): C:\bitirme\results\mc_runs.csv, mc_summary.txt, mc_overview.png
(--n < 100 kısa deneme: build\mc\ altına, tez sonuçlarının üzerine yazmaz)
Bitince ECU varsayılan görüntüsünü geri yükle (CLAUDE.md): fw_build.py --app 4 ... --crc-ref --flash
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = r"C:\bitirme\results"
COLS = ["id", "verdict", "fault", "react_ms", "min_soc", "max_tc", "soc_err", "final_state", "tc_limp", "tc_safe", "why", "run_ms"]

ap = argparse.ArgumentParser()
ap.add_argument("--n", type=int, default=1000)
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--port", default="COM9")
ap.add_argument("--pc-only", action="store_true")
ap.add_argument("--stm-log", help="kart çıktısı kaydı (yeniden koşturmadan karşılaştır)")
a = ap.parse_args()
if a.n < 100:          # kısa deneme tez sonuçlarının üzerine yazmasın
    RES = os.path.join(ROOT, "build", "mc")


def gcc():
    g = shutil.which("gcc") or next(iter(glob.glob(
        r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
    if not g:
        sys.exit("gcc not found")
    return g


def run_pc():
    g = gcc()
    exe = os.path.join(ROOT, "build", "mc_sil.exe")
    env = dict(os.environ, PATH=os.path.dirname(g) + os.pathsep + os.environ["PATH"])
    srcs = ["tools/mc/mc_sil.c", "bms/sim/mc_core.c", "bms/sim/twin_model.c", "bms/common/bms_algo.c"]
    subprocess.run([g, "-O2", "-std=c99", "-Wall", "-Ibms/common", "-Ibms/sim", *srcs, "-lm", "-o", exe],
                   check=True, cwd=ROOT, env=env)
    t0 = time.time()
    out = subprocess.run([exe, str(a.n), str(a.seed)], capture_output=True, text=True, check=True, env=env).stdout
    print(f"PC SIL: {a.n} runs in {time.time() - t0:.1f} s")
    return out.splitlines()


def run_stm():
    import serial
    subprocess.run([sys.executable, os.path.join(ROOT, "tools", "fw_build.py"), "--app", "8",
                    "--define", f"MC_N={a.n}", "--define", f"MC_SEED={a.seed}", "--define", "MC_TRACE_EVERY=0", "--flash"], check=True)
    cli = glob.glob(r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools."
                    r"cubeprogrammer.win32_*\tools\bin\STM32_Programmer_CLI.exe")[0]
    s = serial.Serial(a.port, 115200, timeout=2)
    subprocess.run([cli, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True)
    lines, t0, last = [], time.time(), time.time()
    log = os.path.join(ROOT, "build", "mc_stm.txt")
    with open(log, "w") as f:
        while time.time() - last < 30:
            ln = s.readline().decode(errors="ignore").strip()
            if not ln:
                continue
            last = time.time()
            lines.append(ln)
            f.write(ln + "\n")
            if ln.startswith("RE,") and int(ln.split(",")[1]) % 50 == 0:
                print(f"  board: run {ln.split(',')[1]}/{a.n} ({time.time() - t0:.0f} s)", flush=True)
            if ln.startswith("MC,DONE"):
                break
    print(f"board: {len(lines)} lines -> {log}")
    return lines


def parse(lines):
    rs = {int(l.split(",")[1]): l.split(",", 2)[2] for l in lines if l.startswith("RS,")}
    rows = [l.split(",") for l in lines if l.startswith("RE,")]
    df = pd.DataFrame([r[1:] for r in rows], columns=COLS)
    for c in ("id", "react_ms", "final_state", "run_ms"):
        df[c] = df[c].astype(int)
    for c in ("min_soc", "max_tc", "soc_err", "tc_limp", "tc_safe"):
        df[c] = df[c].astype(float)
    df["scenario"] = df.id.map(rs)
    return df


def summary(pc, stm):
    txt = []
    p = lambda s="": txt.append(s)
    p(f"On-target Monte Carlo validation: {a.n} randomized closed-loop runs (600 s each, Ts 0.1 s), seed {a.seed}")
    p("Plant: 19S12P digital twin (bms/sim/twin_model.c); ECU: bms_step (bms/common), same C code on both targets.")
    p("Randomized per run: T_amb 0-40 C, SOC0 35-95 %, drive severity x0.6-1.6, cycle offset, capacity +-2 %,")
    p("R +-5 %, current-sensor offset +-2.5 A; one fault (none 30 %, cooling 20 %, ext. heat 15 %, NTC open 15 %,")
    p("large current offset 4-12 A 20 %) at a random time 60-480 s.")
    p()
    ref = stm if stm is not None else pc
    g = ref.groupby("fault")
    t = pd.DataFrame({"runs": g.size(), "PASS": g.verdict.apply(lambda v: (v == "PASS").sum()),
                      "react_ms median": g.react_ms.apply(lambda v: np.median(v[v >= 0]) if (v >= 0).any() else np.nan),
                      "react_ms max": g.react_ms.apply(lambda v: v[v >= 0].max() if (v >= 0).any() else np.nan),
                      "detected %": g.react_ms.apply(lambda v: 100 * (v >= 0).mean()),
                      "SOC err max %": g.soc_err.max(), "T_core max C": g.max_tc.max()})
    t.loc["ALL"] = [len(ref), (ref.verdict == "PASS").sum(), np.nan, np.nan, np.nan, ref.soc_err.max(), ref.max_tc.max()]
    h = ref[ref.fault == "ext_heat"]
    if len(h):
        hl, hs = h.tc_limp[h.tc_limp > -90], h.tc_safe[h.tc_safe > -90]
        heat = (f"External heat: true core at LIMP {hl.min():.1f}-{hl.max():.1f} C (median {hl.median():.1f}, limit 55), "
                f"at SAFE {hs.min():.1f}-{hs.max():.1f} C (median {hs.median():.1f}, limit 60)")
    p(f"Results ({'board' if stm is not None else 'PC'}):")
    p(t.round(2).to_string())
    if len(h):
        p(heat)
    fails = ref[ref.verdict != "PASS"]
    p(f"\nFAIL runs: {len(fails)}" + ("" if fails.empty else "\n" + fails[["id", "fault", "why", "scenario"]].to_string(index=False)))
    if stm is not None:
        m = pc.merge(stm, on="id", suffixes=("_pc", "_stm"))
        same = (m.verdict_pc == m.verdict_stm) & (m.fault_pc == m.fault_stm) & (m.final_state_pc == m.final_state_stm)
        exact = same & (m.react_ms_pc == m.react_ms_stm) & (m.min_soc_pc == m.min_soc_stm) \
            & (m.max_tc_pc == m.max_tc_stm) & (m.soc_err_pc == m.soc_err_stm)
        p(f"\nBoard vs PC (same seed, same C code): {len(m)} runs compared")
        p(f"  identical verdict / fault / final state : {same.sum()}/{len(m)}")
        p(f"  identical all printed fields            : {exact.sum()}/{len(m)}")
        p(f"  max |diff| react_ms {np.abs(m.react_ms_pc - m.react_ms_stm).max()} ms, min_soc "
          f"{np.abs(m.min_soc_pc - m.min_soc_stm).max():.1f} %, max_tc {np.abs(m.max_tc_pc - m.max_tc_stm).max():.1f} C, "
          f"soc_err {np.abs(m.soc_err_pc - m.soc_err_stm).max():.2f} %")
        if (~exact).any():
            p("  differing runs: " + ", ".join(str(i) for i in m.id[~exact].head(20)))
        rm = stm.run_ms
        p(f"\nBoard timing (STM32G431 @ 170 MHz, -O2): run mean {rm.mean():.0f} ms, max {rm.max()} ms "
          f"(6000 steps: {1000 * rm.mean() / 6000:.0f} us/step incl. twin), total {rm.sum() / 1000:.0f} s "
          f"= {6000 * 0.1 * len(rm) / (rm.sum() / 1000):.0f}x real time")
    return "\n".join(txt), t


def plot(ref, src):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import thesis_style as ts                       # ortak tez figür stili
    ts.apply()
    fig, ax = plt.subplots(1, 3, figsize=(12.5, 3.9))
    order = ["none", "cooling", "ext_heat", "ntc_open", "i_offset"]
    col = {"none": ts.MUTED, "cooling": ts.BLUE, "ext_heat": ts.ORANGE, "ntc_open": ts.AQUA, "i_offset": ts.YELLOW}
    # (a) PASS oranı
    g = ref.groupby("fault")
    n = g.size().reindex(order).fillna(0)
    pas = g.verdict.apply(lambda v: (v == "PASS").sum()).reindex(order).fillna(0)
    ax[0].bar(order, n, color=ts.LIGHT)
    ax[0].bar(order, pas, color=[col[o] for o in order])
    for i, o in enumerate(order):
        ax[0].text(i, n[o], f"{int(pas[o])}/{int(n[o])}", ha="center", va="bottom", fontsize=8)
    ax[0].set_ylabel("runs (colored = PASS)")
    ax[0].set_title("(a) verdict per fault type", loc="left", fontsize=9)
    ax[0].tick_params(axis="x", labelsize=8, rotation=20)
    # (b) tepki süresi
    for i, o in enumerate(order[1:]):
        v = ref.react_ms[(ref.fault == o) & (ref.react_ms >= 0)] / 1000
        if len(v):
            ax[1].scatter(np.full(len(v), i) + np.random.default_rng(0).uniform(-0.25, 0.25, len(v)), v.clip(lower=0.05),
                          s=8, color=col[o], alpha=0.6)
    ax[1].set_yscale("log")
    ax[1].set_xticks(range(4), order[1:], fontsize=8, rotation=20)
    ax[1].set_ylabel("reaction time after fault [s]")
    ax[1].text(2, 0.07, "same sample (0 s)", ha="center", fontsize=7.5)
    ax[1].set_title("(b) reaction time (log)", loc="left", fontsize=9)
    # (c) dış ısı: LIMP / SAFE anında gerçek çekirdek sıcaklığı (ECU yüzeyi ölçer, çekirdeği kestirir)
    h = ref[ref.fault == "ext_heat"]
    bins = np.arange(52.0, 62.01, 0.2)
    ax[2].hist(h.tc_limp[h.tc_limp > -90], bins=bins, color=ts.SERIOUS, label="at LIMP-HOME")
    ax[2].hist(h.tc_safe[h.tc_safe > -90], bins=bins, color=ts.CRITICAL, label="at SAFE-STOP")
    for lim, lab in ((55, "LIMP limit 55 °C"), (60, "SAFE limit 60 °C")):
        ax[2].axvline(lim, color=ts.MUTED, lw=0.8, ls="--")
        ax[2].text(lim + 0.1, ax[2].get_ylim()[1] * 0.95, lab, fontsize=7.5, va="top")
    ax[2].set_xlabel("true core temperature at the transition [°C]")
    ax[2].set_ylabel("ext_heat runs")
    ax[2].set_title(f"(c) external heat: true core at LIMP / SAFE (n = {len(h)})", loc="left", fontsize=9)
    ax[2].legend(frameon=False, fontsize=7.5, loc="upper left")
    for x in ax:
        x.grid(axis="x", visible=False)
    fig.suptitle(f"On-target Monte Carlo: {len(ref)} randomized closed-loop runs on STM32G431 ({src})",
                 x=0.01, ha="left", fontsize=10, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(RES, "mc_overview.png"), dpi=200)


pc = parse(run_pc())
stm = None
if a.stm_log:
    stm = parse(open(a.stm_log).read().splitlines())
elif not a.pc_only:
    stm = parse(run_stm())
txt, _ = summary(pc, stm)
print(txt)
os.makedirs(RES, exist_ok=True)
out = pc.merge(stm, on="id", suffixes=("_pc", "_board")) if stm is not None else pc
out.to_csv(os.path.join(RES, "mc_runs.csv"), index=False)
open(os.path.join(RES, "mc_summary.txt"), "w", encoding="utf-8").write(txt + "\n")
plot(stm if stm is not None else pc, "board" if stm is not None else "PC SIL")
print(f"-> {RES}\\mc_runs.csv, mc_summary.txt, mc_overview.png")
