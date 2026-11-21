"""Senaryo koşturucu (demo + otomatik HIL testi): stres -> ECU fark eder -> önlem alır.

    python tools/demo_run.py tools/demo/stres.txt [--emu COM10] [--hiz 1]

Uno'da twin_emu, ECU'da --app 4 BMS_SAFETY=1 BMS_OUTPUTS=1 olmalı. COM10 açılınca Uno sıfırlanır;
ECU SWD ile sıfırlanır (önce EMU, sonra ECU). ECU'nun durumu Uno'nun 1 s'lik R satırından okunur
(0x201/0x202), yani COM9 boş kalır.

Senaryo dosyası, satır satır ('#' yorum):
    +<s> <komut> ; açıklama      s saniye bekle, komutu Uno'ya gönder (twin_emu seri komutları:
                                 p, t, a, f, h, m, s, c ...), açıklamayı yaz
    +<s> !reset ; açıklama       ECU'yu SWD ile sıfırla (SAFE/LIMP kilidinden çıkış, gözlemciler sıfırdan)    +<s> - ; açıklama            yalnız bekle ve yaz
    bekle <ifade> <en çok s> ; açıklama
                                 ifade doğru olana kadar bekle (değişkenler: state, Idis, Ichg,
                                 cause, Tc, Ts, tc_ecu, I, v); olmazsa "ZAMAN AŞIMI" yazar, sürer
    === <başlık>                 bölüm başlığı (grafikte dikey çizgi)
Durum ya da neden değişince satır basılır. Çıktı: build/demo_<ad>.csv ve .png (tez figürü).
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

import serial

# yönlendirilmiş çıktıda (Windows cp1252) Türkçe karakter çökmesin
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ap = argparse.ArgumentParser()
ap.add_argument("senaryo")
ap.add_argument("--emu", default="COM10")
a = ap.parse_args()

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ad = os.path.splitext(os.path.basename(a.senaryo))[0]
CSV = os.path.join(ROOT, "build", f"demo_{ad}.csv")
PNG = os.path.join(ROOT, "build", f"demo_{ad}.png")
os.makedirs(os.path.dirname(CSV), exist_ok=True)
CLI = sorted(glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*cubeprogrammer*\tools\bin\STM32_Programmer_CLI.exe"))[-1]
ST = {0: "----", 1: "NORMAL", 2: "DERATE", 3: "LIMP", 4: "SAFE"}
NEDEN = [(1, "sicaklik"), (2, "sogutma alarmi"), (4, "desarj gerilim payi"), (8, "sarj gerilim payi"),
         (16, "veri sayfasi sicaklik siniri"), (32, "sensor arizasi"), (64, "bos batarya"), (128, "sert sinir")]


def neden_str(c):
    return ", ".join(n for b, n in NEDEN if c & b) or "-"


# ------------------------------------------------------------------ senaryo
adimlar = []
for no, ln in enumerate(open(a.senaryo, encoding="utf-8"), 1):
    ln = ln.strip()
    if not ln or ln.startswith("#"):
        continue
    if ln.startswith("==="):
        adimlar.append(("baslik", ln[3:].strip()))
        continue
    govde, _, acik = ln.partition(";")
    govde, acik = govde.strip(), acik.strip()
    if govde.startswith("+"):
        s, _, kmt = govde[1:].partition(" ")
        adimlar.append(("komut", float(s), kmt.strip(), acik))
    elif govde.startswith("bekle "):
        ifade, maks = govde[6:].rsplit(" ", 1)
        adimlar.append(("bekle", ifade.strip(), float(maks), acik))
    else:
        sys.exit(f"{a.senaryo}:{no}: anlaşılmadı: {ln}")

# ------------------------------------------------------------------ bağlantı
emu = serial.Serial(a.emu, 115200, timeout=0.1)
time.sleep(3.0)                                   # Uno sıfırlanır ve açılır
emu.reset_input_buffer()
t0 = time.time()
rows, olaylar, buf = [], [], b""
son = {"state": 0, "Idis": 0.0, "Ichg": 0.0, "cause": 0, "Tc": 25.0, "Ts": 25.0, "tc_ecu": 25.0, "I": 0.0, "v": 0.0}
son_anahtar = [None]
f = open(CSV, "w")
f.write("t,state,Idis,Ichg,cause,Tc,Ts,tc_ecu,I,v,soc,soc_ecu\n")


def zaman():
    return time.time() - t0


def yaz(s):
    print(f"[{zaman():6.1f} s] {s}", flush=True)


def pompala(sure):
    """sure boyunca Uno'yu oku; R satırlarını işle."""
    global buf
    bitis = time.time() + sure
    while True:
        buf += emu.read(512)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            t = ln.decode(errors="replace").strip()
            if not t.startswith("R,"):
                continue
            p = t.split(",")
            if len(p) < 30:
                continue
            try:
                son.update(state=int(p[11]), Idis=float(p[12]), Ichg=float(p[13]), cause=int(p[14]),
                           Tc=float(p[7]), Ts=float(p[8]), tc_ecu=float(p[16]), I=float(p[4]), v=float(p[3]))
            except ValueError:
                continue
            taze = int(p[17]) > 0 and int(p[19]) < 1000          # err alanına 0x201 zaman aşımında +1000
            st = son["state"] if taze else 0
            son["state"] = st                                     # bekle ifadeleri de 0 = mesaj yok görür
            rows.append((round(zaman(), 1), st, son["Idis"], son["Ichg"], son["cause"], son["Tc"], son["Ts"],
                         son["tc_ecu"], son["I"], son["v"], float(p[6]), float(p[15])))
            f.write(",".join(str(x) for x in rows[-1]) + "\n")
            anahtar = (st, son["cause"])
            if anahtar != son_anahtar[0]:
                son_anahtar[0] = anahtar
                yaz(f"   ECU: {ST.get(st, st):6s} izin {son['Idis']:5.1f} A | neden: {neden_str(son['cause'])}"
                    f" | Tc^ {son['tc_ecu']:.1f} C")
        if time.time() >= bitis:
            return


def ecu_reset():
    subprocess.run([CLI, "-c", "port=SWD", "mode=HOTPLUG", "-rst"], capture_output=True)


ecu_reset()
yaz("ECU sifirlandi, senaryo basliyor")
for ad_ in adimlar:
    if ad_[0] == "baslik":
        yaz(f"===== {ad_[1]} =====")
        olaylar.append((zaman(), ad_[1]))
    elif ad_[0] == "komut":
        _, s, kmt, acik = ad_
        pompala(s)
        if kmt == "!reset":
            ecu_reset()
            pompala(3.0)          # eski (sıfırlama öncesi) durumun R satırları geçsin
        elif kmt and kmt != "-":
            emu.write((kmt + "\n").encode())
        if acik:
            yaz(acik)
    else:
        _, ifade, maks, acik = ad_
        bas = zaman()
        while True:
            pompala(0.2)
            try:
                if eval(ifade, {}, dict(son)):
                    break
            except Exception as e:
                sys.exit(f"bekle ifadesi hatalı: {ifade}: {e}")
            if zaman() - bas > maks:
                yaz(f"ZAMAN ASIMI ({maks:.0f} s): {ifade}")
                break
        if acik:
            yaz(f"{acik}  ({zaman() - bas:.0f} s sonra)")
pompala(2.0)
f.close()
emu.close()
yaz(f"bitti -> {os.path.relpath(CSV, ROOT)}")

# ------------------------------------------------------------------ grafik
try:
    import pandas as pd
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import thesis_style as ts                       # ortak tez figür stili
    ts.apply()
    d = pd.read_csv(CSV)
    fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    ax[0].plot(d.t, d.I, label="Pack current I")
    ax[0].plot(d.t, d.Idis, label="Allowed discharge I_allow")
    ax[0].set_ylabel("A"); ax[0].legend(loc="upper right")
    ax[1].plot(d.t, d.Tc, label="True core T (twin)")
    ax[1].plot(d.t, d.tc_ecu, label="ECU estimate")
    ax[1].set_ylabel("°C"); ax[1].legend(loc="upper right")
    ax[2].step(d.t, d.state, where="post", color=ts.INK2, lw=1)
    ts.state_band(ax[2], d.t, d.state, legend=False)
    ax[2].set_yticks([0, 1, 2, 3, 4]); ax[2].set_yticklabels(["no msg", "NORMAL", "DERATE", "LIMP", "SAFE"])
    ax[2].set_xlabel("time [s]")
    for t, b in olaylar:
        for x in ax:
            x.axvline(t, color=ts.MUTED, lw=0.6, ls=":")
        ax[0].text(t, ax[0].get_ylim()[1], " " + b, fontsize=7, va="top", rotation=90, color=ts.INK2)
    fig.tight_layout(); fig.savefig(PNG, dpi=200)
    yaz(f"grafik -> {os.path.relpath(PNG, ROOT)}")
except Exception as e:
    yaz(f"grafik cizilemedi: {e}")
