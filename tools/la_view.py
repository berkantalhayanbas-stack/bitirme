"""
Adım 5: STM32 lojik analizörünün (APP_LA, bms/pil/app_la.c) PC tarafı.

Kartla 'C' gönderilir, PA11 (CAN RX) için 16 384 örnek (4,25 MHz, 3,855 ms) alınır.
Çözüm: koşu uzunlukları (aynı seviyedeki ardışık örnekler) bit sayısına yuvarlanır
(bit = 8,5 örnek; doldurma kuralı yüzünden koşu en çok 5 bit, yuvarlama payı ±4 örnek), her çerçeve:
SOF | ID (11) | RTR | IDE | r0 | DLC (4) | veri | CRC (15) | CRC del | ACK | ACK del | EOF (7);
SOF..CRC arası bit doldurma (5 aynı bitten sonra ters bit) çözülür, CRC-15 (0x4599) doğrulanır.
Bit süresi: çerçevedeki tüm kenar zamanlarının bit indisine doğrusal uydurması (eğim = bit süresi);
örnekleme saati STM32'nin HSI tabanlı PLL'i, bitleri Arduino'nun MCP2515 kristali üretir: ölçüm
iki saatin oranını da içerir. Beklenen 2,00 µs (500 kbit/s).

Kullanım:
  python tools/la_view.py --port COM9                    # yakala, çöz, build/la/la_capture.png
  python tools/la_view.py --port COM9 --out x.png --save raw.npy
  python tools/la_view.py --load raw.npy                 # kartsız, kayıttan
"""
import argparse
import os
import struct
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BITRATE = 500_000


def crc8_j1850(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def capture(port):
    import serial
    s = serial.Serial(port, 115200, timeout=3.0)
    s.reset_input_buffer()
    s.write(b"C")
    # "LA" başlığını ara (açılış mesajı vb. atlanır)
    buf = b""
    t_end = time.time() + 5
    while not buf.endswith(b"LA"):
        c = s.read(1)
        if not c or time.time() > t_end:
            sys.exit("no answer from the board (APP_LA flashed? port free?)")
        buf = (buf + c)[-2:]
    hdr = s.read(8)
    rate, n, bit, status = struct.unpack("<IHBB", hdr)
    data = s.read(n // 8)
    crc = s.read(1)
    if len(data) != n // 8 or not crc or crc8_j1850(hdr + data) != crc[0]:
        sys.exit("corrupted block from the board")
    if status != 0:
        sys.exit("no trigger: no CAN frame within 2 s (is the other node sending?)")
    bits = np.unpackbits(np.frombuffer(data, np.uint8))
    return rate, bits


def runs_of(x):
    """(seviye, başlangıç, uzunluk) listesi"""
    edges = np.flatnonzero(np.diff(x)) + 1
    starts = np.concatenate(([0], edges))
    lens = np.diff(np.concatenate((starts, [len(x)])))
    return [(int(x[s]), int(s), int(l)) for s, l in zip(starts, lens)]


def crc15(bits):
    crc = 0
    for b in bits:
        nxt = b ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    return crc


def decode(rate, x):
    """Örneklerden çerçeveler. Dönüş: bit zaman çizelgesi ve çerçeve listesi."""
    spb = rate / BITRATE
    # örnek düzeyindeki koşuları bitlere çevir: her bitin (seviye, başlangıç örneği)
    bits, t_bits = [], []
    for lvl, s, l in runs_of(x):
        k = max(1, int(round(l / spb)))
        for j in range(k):
            bits.append(lvl)
            t_bits.append(s + j * spb)
    bits = np.array(bits)
    frames, i = [], 0
    while i < len(bits):
        if bits[i] != 0:                                   # SOF ara
            i += 1
            continue
        f = parse_frame(bits, i)
        if f is None:
            break
        f["t0"] = t_bits[i]
        f["t_bits"] = [t_bits[k] for k in f["raw_idx"]]
        frames.append(f)
        i = f["end"]
        while i < len(bits) and bits[i] == 1:              # boşluk
            i += 1
    return bits, np.array(t_bits), frames, spb


def parse_frame(bits, i0):
    """i0 = SOF indisi. Destuff + alan çözümü. raw_idx/field: ham bit başına alan adı."""
    field_of, raw_idx, d = [], [], []
    run_lvl, run_n = None, 0
    i = i0
    need = None                                            # SOF..CRC sonu (doldurmalı kısım)
    names = []

    def name_for(k):                                       # destuff edilmiş bit indisi -> alan
        if k == 0: return "SOF"
        if k <= 11: return "ID"
        if k == 12: return "RTR"
        if k == 13: return "IDE"
        if k == 14: return "r0"
        if k <= 18: return "DLC"
        nd = 8 * min(int("".join(map(str, d[15:19])), 2), 8) if len(d) >= 19 else 0
        if k <= 18 + nd: return "DATA"
        return "CRC"

    n_stuff = 0
    while True:
        if i >= len(bits):
            return None
        b = int(bits[i])
        if run_n == 5:                                     # doldurma biti bekleniyor
            if b == run_lvl:
                return dict(error=f"stuff error at bit {i - i0}", end=i + 1, raw_idx=raw_idx,
                            field=field_of, id=None)
            field_of.append("stuff"); raw_idx.append(i); n_stuff += 1
            run_lvl, run_n = b, 1
            i += 1
            continue
        k = len(d)
        d.append(b)
        field_of.append(name_for(k)); raw_idx.append(i)
        run_n = run_n + 1 if b == run_lvl else 1
        run_lvl = b
        i += 1
        if len(d) == 19:
            dlc = int("".join(map(str, d[15:19])), 2)
            need = 19 + 8 * min(dlc, 8) + 15
        if need is not None and len(d) == need:
            break
    # doldurmasız kuyruk: CRC del, ACK, ACK del, EOF 7
    tail = ["CRC del", "ACK", "ACK del"] + ["EOF"] * 7
    for nm in tail:
        if i >= len(bits):
            return None
        field_of.append(nm); raw_idx.append(i)
        i += 1
    ident = int("".join(map(str, d[1:12])), 2)
    dlc = int("".join(map(str, d[15:19])), 2)
    nb = min(dlc, 8)
    data = bytes(int("".join(map(str, d[19 + 8 * j:27 + 8 * j])), 2) for j in range(nb))
    crc_rx = int("".join(map(str, d[19 + 8 * nb:34 + 8 * nb])), 2)
    crc_calc = crc15(d[:19 + 8 * nb])
    tb = [int(bits[k]) for k in raw_idx]
    ack = tb[field_of.index("ACK")] == 0
    eof_ok = all(tb[j] == 1 for j, f in enumerate(field_of) if f in ("EOF", "CRC del", "ACK del"))
    return dict(id=ident, dlc=dlc, data=data, crc_rx=crc_rx, crc_calc=crc_calc, ack=ack, eof_ok=eof_ok,
                rtr=d[12], ide=d[13], n_stuff=n_stuff, n_bits=len(raw_idx), end=i, raw_idx=raw_idx,
                field=field_of, error=None)


def bit_time(frames, t_bits, bits, spb, rate):
    """Kenar zamanlarının bit indisine doğrusal uydurması -> bit süresi [µs]."""
    # Kenar zamanı = koşunun ilk örneği (gerçek kenar; sonraki bitlerin zamanı türetilmiş).
    # Çerçeveler arası uzun boşluk bit sayısına yuvarlanırken ±0,5 bit kayabilir: her çerçeveye
    # ayrı kesişim, ortak eğim (merkezlenmiş en küçük kareler).
    sxy = sxx = 0.0
    n = 0
    for f in frames:
        idx = f["raw_idx"]
        xs, ys = [], []
        for a, b in zip(idx[:-1], idx[1:]):
            if bits[a] != bits[b]:                         # SOF'un kendisi yakalama başı: kenar sayılmaz
                xs.append(b)
                ys.append(t_bits[b])
        if len(xs) < 2:
            continue
        xs, ys = np.array(xs, float), np.array(ys, float)
        xc, yc = xs - xs.mean(), ys - ys.mean()
        sxy += float(xc @ yc)
        sxx += float(xc @ xc)
        n += len(xs)
    if sxx == 0:
        return float("nan"), 0
    return sxy / sxx / rate * 1e6, n


def plot(rate, x, bits, t_bits, frames, spb, out, tb_us):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t_us = np.arange(len(x)) / rate * 1e6
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(15, 7.5), gridspec_kw=dict(height_ratios=[1, 2.2]))
    a1.step(t_us, x, where="post", lw=0.7, color="k")
    for f in frames:
        t0 = f["t0"] / rate * 1e6
        a1.axvline(t0, color="tab:blue", lw=0.8, ls=":")
        a1.text(t0, 1.15, f"0x{f['id']:03X}" if f["id"] is not None else "?", fontsize=8, color="tab:blue")
    a1.set_ylim(-0.3, 1.45); a1.set_yticks([0, 1]); a1.set_yticklabels(["dom", "rec"])
    a1.set_xlabel("Time [µs]"); a1.set_title(f"PA11 (CAN RX) captured by the STM32 itself: TIM6-triggered DMA "
                                             f"of GPIOA->IDR, {rate / 1e6:.2f} MS/s, {len(x)} samples")
    f = frames[0]
    i0, i1 = max(int(f["t_bits"][0]) - 10, 0), int(f["t_bits"][-1] + spb) + 10
    sl = slice(i0, min(i1, len(x)))
    a2.step(t_us[sl], x[sl], where="post", lw=1.0, color="k")
    colors = {"SOF": "#c7e9c0", "ID": "#9ecae1", "RTR": "#fdd0a2", "IDE": "#fdd0a2", "r0": "#fdd0a2",
              "DLC": "#dadaeb", "DATA": "#fee391", "CRC": "#fcbba1", "CRC del": "#e0e0e0", "ACK": "#a1d99b",
              "ACK del": "#e0e0e0", "EOF": "#f0f0f0", "stuff": "#fb6a4a"}
    # alanları birleştirerek göster
    segs = []
    for k, (ri, fld) in enumerate(zip(f["raw_idx"], f["field"])):
        t = t_bits[ri]
        if segs and segs[-1][0] == fld and fld != "stuff":
            segs[-1][2] = t + spb
        else:
            segs.append([fld, t, t + spb])
    seen, narrow = set(), 0
    for fld, ta, tb in segs:
        a2.axvspan(ta / rate * 1e6, tb / rate * 1e6, color=colors.get(fld, "#ffffff"), alpha=0.8, lw=0)
        if fld == "stuff" or (fld in seen and fld in ("ID", "DATA", "CRC")):
            continue                                       # doldurmayla bölünen alanı bir kez yaz
        seen.add(fld)
        y = 1.18
        if tb - ta < 1.5 * spb:                            # 1 bitlik alanlar: kademeli yükseklik
            y = (1.10, 1.20, 1.30)[narrow % 3]
            narrow += 1
        a2.text((ta + tb) / 2 / rate * 1e6, y, fld, ha="center", fontsize=7.5)
    stuff_t = [t_bits[ri] for ri, fld in zip(f["raw_idx"], f["field"]) if fld == "stuff"]
    a2.plot([(t + spb / 2) / rate * 1e6 for t in stuff_t], [-0.15] * len(stuff_t), "v", color="#cb181d",
            ms=6, label=f"stuff bits ({len(stuff_t)})")
    data = " ".join(f"{b:02X}" for b in f["data"])
    a2.set_title(f"Frame 1 decoded: ID 0x{f['id']:03X}, DLC {f['dlc']}, data [{data}], "
                 f"CRC 0x{f['crc_rx']:04X} ({'OK' if f['crc_rx'] == f['crc_calc'] else 'BAD'}), "
                 f"ACK {'yes' if f['ack'] else 'no'}  |  bit time {tb_us:.3f} µs (expected 2.000)")
    a2.set_ylim(-0.35, 1.35); a2.set_yticks([0, 1]); a2.set_yticklabels(["dominant", "recessive"])
    a2.set_xlabel("Time [µs]"); a2.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"figure: {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "la", "la_capture.png"))
    ap.add_argument("--save", help="save raw samples (.npy, with rate)")
    ap.add_argument("--load", help="decode a saved capture instead of the board")
    a = ap.parse_args()
    if a.load:
        z = np.load(a.load)
        rate, x = int(z[0]), z[1:].astype(np.uint8)
    else:
        rate, x = capture(a.port)
        if a.save:
            np.save(a.save, np.concatenate(([rate], x)).astype(np.int64))
    bits, t_bits, frames, spb = decode(rate, x)
    good = [f for f in frames if f.get("error") is None]
    print(f"capture: {len(x)} samples at {rate / 1e6:.3f} MS/s ({len(x) / rate * 1e3:.3f} ms), "
          f"{spb:.2f} samples/bit, {len(frames)} frame(s)")
    print("RESULT | # | ID | DLC | data | CRC rx | CRC calc | CRC | ACK | stuff bits | frame bits | EOF")
    for n, f in enumerate(frames, 1):
        if f.get("error"):
            print(f"RESULT | {n} | ERROR: {f['error']}")
            continue
        print(f"RESULT | {n} | 0x{f['id']:03X} | {f['dlc']} | {f['data'].hex(' ').upper()} | 0x{f['crc_rx']:04X} | "
              f"0x{f['crc_calc']:04X} | {'OK' if f['crc_rx'] == f['crc_calc'] else 'BAD'} | "
              f"{'yes' if f['ack'] else 'NO'} | {f['n_stuff']} | {f['n_bits']} | {'OK' if f['eof_ok'] else 'BAD'}")
    tb_us, n_edges = bit_time(good, t_bits, bits, spb, rate)
    print(f"RESULT | bit time {tb_us:.4f} us from {n_edges} edges (expected 2.0000) -> {1 / tb_us * 1e3:.2f} kbit/s")
    ok = good and all(f["crc_rx"] == f["crc_calc"] and f["ack"] and f["eof_ok"] for f in good) \
        and len(good) == len(frames) and abs(tb_us - 2.0) < 0.02
    print(f"RESULT | verdict {'PASS' if ok else 'FAIL'} (all CRC-15 OK, ACKed, EOF OK, bit time within 1 %)")
    if good:
        plot(rate, x, bits, t_bits, good, spb, a.out, tb_us)


if __name__ == "__main__":
    main()
