r"""Ham CAN kaydı: ECU'nun (APP_ECU, BMS_CANLOG=1) UART'a yazdığı "C,<t_ms>,<R|T>,<id>,<veri>" satırlarını
candump biçimine çevirir (SavvyCAN: File > Load Log File > Candump/Kayak; SocketCAN canplayer).
ECU filtresinden geçen her çerçeve (0x101, 0x102, 0x110-0x114, 0x301) R, ECU'nun gönderdikleri
(0x201, 0x202, 0x210) T olarak gelir; hattaki tüm trafik budur. Zaman damgası ECU'nun 1 ms saati.

Kayıt sonunda özet (ad bms.dbc'den): kimlik başına sayı/periyot, CRC-8 ve sayaç hataları, bit doldurma
dahil gerçek çerçeve bitinden hat yükü. frame_bits() lojik analizörün 246 çerçevesiyle
(build/la/la_stats.csv, bits = frame_bits + 3) birebir aynı.

    python tools/fw_build.py --app 4 --define BMS_CANLOG=1 --flash
    python tools/can/can_log.py --dur 60                       # COM9, build/can_log.log
    python tools/can/can_log.py --load build/can_log_raw.txt   # kartsız, ham UART kaydından
    python tools/can/can_log.py --sil heat --dur 300           # kartsız, SIL (can_sil.c), build/can_sil_heat.log
"""
import argparse, os, sys, time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
BITRATE = 500_000


def crc8_j1850(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def frame_bits(cid, data):
    """Standart veri çerçevesinin hattaki bit sayısı (doldurma dahil, çerçeveler arası 3 bit hariç)."""
    bits = [0] + [(cid >> (10 - i)) & 1 for i in range(11)] + [0, 0, 0]          # SOF ID RTR IDE r0
    bits += [(len(data) >> (3 - i)) & 1 for i in range(4)]
    for b in data:
        bits += [(b >> (7 - i)) & 1 for i in range(8)]
    crc = 0
    for bit in bits:                                                               # CRC-15, 0x4599
        nxt = bit ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    bits += [(crc >> (14 - i)) & 1 for i in range(15)]
    stuffed, run, last = 0, 0, None
    for bit in bits:
        run = run + 1 if bit == last else 1
        last = bit
        if run == 5:
            stuffed += 1
            run, last = 1, 1 - bit                                                 # eklenen ters bit
    return len(bits) + stuffed + 1 + 2 + 7                                         # CRC del, ACK+del, EOF


def parse(line):
    p = line.strip().split(",")
    if len(p) != 5 or p[0] != "C":
        return None
    try:
        return int(p[1]), p[2], int(p[3], 16), bytes.fromhex(p[4])
    except ValueError:
        return None


def read_port(port, dur, raw_path):
    import serial
    frames = []
    with serial.Serial(port, 115200, timeout=0.2) as s, open(raw_path, "w", newline="") as raw:
        s.reset_input_buffer()
        t_end = time.time() + dur
        buf = b""
        while time.time() < t_end:
            buf += s.read(4096)
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                txt = ln.decode("ascii", "replace").rstrip("\r")
                raw.write(txt + "\n")
                f = parse(txt)
                if f:
                    frames.append(f)
            sys.stdout.write(f"\r{len(frames)} frames, {t_end - time.time():5.1f} s left ")
            sys.stdout.flush()
    print()
    return frames


def write_pcapng(path, frames, t0, iface):
    """Wireshark için pcapng: LINKTYPE_CAN_SOCKETCAN (227), çerçeve başına 16 bayt (can_id büyük
    endian, len, 3 ayrılmış, 8 veri). EPB flags yönü taşır: R = gelen (1), T = giden (2), ECU'nun
    gözünden; Wireshark'ta frame.packet_flags_direction. Zaman: ECU saati, 1 ms."""
    import struct

    def block(btype, body):
        body += b"\0" * (-len(body) % 4)
        n = 12 + len(body)
        return struct.pack("<II", btype, n) + body + struct.pack("<I", n)

    def opt(code, val):
        return struct.pack("<HH", code, len(val)) + val + b"\0" * (-len(val) % 4)

    shb = struct.pack("<IHHq", 0x1A2B3C4D, 1, 0, -1) + opt(4, b"bms_ecu tools/can/can_log.py") + opt(0, b"")
    idb = struct.pack("<HHI", 227, 0, 0) + opt(2, iface.encode()) + opt(9, b"\x06") + opt(0, b"")   # 10^-6 s
    with open(path, "wb") as f:
        f.write(block(0x0A0D0D0A, shb) + block(1, idb))
        for t, dr, cid, d in frames:
            us = (t - t0) * 1000
            pkt = struct.pack(">IB3x", cid, len(d)) + d.ljust(8, b"\0")
            flags = opt(2, struct.pack("<I", 1 if dr == "R" else 2)) + opt(0, b"")
            f.write(block(6, struct.pack("<IIIII", 0, us >> 32, us & 0xFFFFFFFF, len(pkt), len(pkt)) + pkt + flags))


def run_sil(scenario, dur, extra):
    """tools/can/can_sil.c'yi bms/common + bms/sim ile derler, koşturur, C, satırlarını döndürür."""
    import glob, shutil, subprocess, tempfile
    gcc = shutil.which("gcc") or next(iter(glob.glob(
        r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
    if not gcc:
        sys.exit("gcc not found")
    com, sim = os.path.join(ROOT, "bms", "common"), os.path.join(ROOT, "bms", "sim")
    srcs = [os.path.join(HERE, "can_sil.c"), os.path.join(sim, "twin_model.c")] + \
           [os.path.join(com, f) for f in ("bms_algo.c", "bms_bal.c", "bms_can.c", "soh_tree.c", "stress_model.c")]
    exe = os.path.join(tempfile.mkdtemp(), "can_sil.exe")
    env = dict(os.environ, PATH=os.path.dirname(gcc) + os.pathsep + os.environ["PATH"])
    subprocess.run([gcc, "-O2", "-std=c99", "-I", com, "-I", sim, *srcs, "-lm", "-o", exe], check=True, env=env)
    out = subprocess.run([exe, str(dur), scenario, *extra], capture_output=True, text=True, check=True, env=env)
    return out.stdout.splitlines()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("--dur", type=float, default=60.0)
    ap.add_argument("--load", help="ham UART kaydı (C, satırları); verilirse port açılmaz")
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "can_log.log"))
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--sil", choices=["drive", "park", "ntc", "heat"],
                    help="kartsız: can_sil.c kapalı çevrimi (ideal hat zamanlaması)")
    ap.add_argument("--soc0", help="SIL başlangıç SOC'si (0-1)")
    ap.add_argument("--tamb", help="SIL ortam sıcaklığı [°C]")
    a = ap.parse_args()
    if a.sil and a.out == ap.get_default("out"):
        a.out = os.path.join(ROOT, "build", f"can_sil_{a.sil}.log")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    if a.sil:
        extra = [a.soc0 or ("0.6" if a.sil == "park" else "0.8")] + ([a.tamb] if a.tamb else [])
        frames = [x for x in map(parse, run_sil(a.sil, a.dur, extra)) if x]
    elif a.load:
        with open(a.load) as f:
            frames = [x for x in map(parse, f) if x]
    else:
        frames = read_port(a.port, a.dur, os.path.splitext(a.out)[0] + "_raw.txt")
    if not frames:
        sys.exit("no C, lines (firmware built with --define BMS_CANLOG=1?)")

    # candump -L biçimi: (sn.mikrosn) arayüz ID#VERİ ; ECU saati 1 ms, sıfır ilk çerçeve
    frames.sort(key=lambda f: f[0])                                                # kararlı: aynı ms'de sıra korunur
    t0 = frames[0][0]
    with open(a.out, "w", newline="\n") as f:
        for t, _, cid, d in frames:
            dt = t - t0
            f.write(f"({dt // 1000}.{(dt % 1000) * 1000:06d}) {a.iface} {cid:03X}#{d.hex().upper()}\n")
    pcap = os.path.splitext(a.out)[0] + ".pcapng"
    write_pcapng(pcap, frames, t0, a.iface)

    # özet: kimlik başına sayı, periyot, CRC/sayaç, hat yükü
    names = {}                                                                     # bms.dbc "BO_ <id> <ad>:"
    with open(os.path.join(HERE, "bms.dbc")) as f:
        for ln in f:
            if ln.startswith("BO_ "):
                p = ln.split()
                names[int(p[1])] = p[2].rstrip(":")
    span = max((frames[-1][0] - t0) / 1000.0, 1e-3)
    by = defaultdict(list)
    for t, dr, cid, d in frames:
        by[cid].append((t, dr, d))
    print(f"{len(frames)} frames in {span:.1f} s -> {a.out}")
    print(f"{'ID':>5} {'dir':>3} {'name':<11} {'n':>6} {'period_ms':>9} {'frame_bits':>10} {'crc_err':>7} {'cnt_err':>7}")
    tot_bits = 0
    for cid in sorted(by):
        rows = by[cid]
        name = names.get(cid, "?")
        per = (rows[-1][0] - rows[0][0]) / (len(rows) - 1) if len(rows) > 1 else 0.0
        bits = [frame_bits(cid, d) for _, _, d in rows]
        tot_bits += sum(bits) + 3 * len(bits)                                      # + çerçeveler arası boşluk
        # CRC kapsamı: ID düşük bayt, ID yüksek bayt, veri 0..6 (bms_can.c, DBC yorumu)
        idb = bytes([cid & 0xFF, cid >> 8])
        crc_err = sum(1 for _, _, d in rows if len(d) == 8 and crc8_j1850(idb + d[:7]) != d[7])
        cnt_err, last = 0, None                                                    # adım 1 dışı = kayıp/tekrar
        for _, _, d in rows:
            c = d[6] & 0x0F if len(d) == 8 else None
            if last is not None and c is not None and c != (last + 1) & 0x0F and cid != 0x301:
                cnt_err += 1
            last = c
        print(f"0x{cid:03X} {rows[0][1]:>3} {name:<11} {len(rows):>6} {per:>9.1f} "
              f"{min(bits):>5}-{max(bits):<4} {crc_err:>7} {cnt_err:>7}")
    print(f"bus load {100.0 * tot_bits / span / BITRATE:.2f} % of {BITRATE // 1000} kbit/s "
          f"(stuffing exact, 3-bit interframe)")


if __name__ == "__main__":
    main()
