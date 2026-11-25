r"""bms_dbc.lua == bms/common/bms_can.c kontrolü (dbc_check.py'nin Wireshark karşılığı): C kodlayıcısının
14000 rastgele çerçevesi pcapng'ye yazılır, tshark + bms_dbc.lua ile çözülür; fiziksel değerler yarım LSB
içinde, durum/neden/bit alanları ve sayaç birebir, CRC-8 her çerçevede "ok" olmalı.
Ayrıca bozulmuş kopyalar (CRC baytı, donmuş sayaç, 300 ms boşluk) expert info üretmeli.

    python tools/can/lua_check.py          # Wireshark (tshark) kurulu olmalı
"""
import glob, json, os, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
from can_log import write_pcapng  # noqa: E402

TSHARK = shutil.which("tshark") or r"C:\Program Files\Wireshark\tshark.exe"
LUA = os.path.join(HERE, "bms_dbc.lua")


def near(a, b, tol):
    return abs(a - b) <= tol * 1.001 + 1e-6


def decode(pcap):
    # kişisel eklenti klasöründeki (kurulmuş) kopya yüklenmesin: depodaki dosya sınanır
    env = dict(os.environ, APPDATA=tempfile.mkdtemp())
    out = subprocess.run([TSHARK, "-X", "lua_script:" + LUA, "-r", pcap, "-T", "json", "-J", "bms"],
                         capture_output=True, text=True, encoding="utf-8", check=True, env=env).stdout
    res = []
    for pkt in json.loads(out):
        lay = pkt["_source"]["layers"]
        flat = {}

        def walk(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    if k.startswith("bms.") and not isinstance(v, (dict, list)):
                        flat[k[4:]] = v
                    walk(v)
            elif isinstance(o, list):
                for v in o:
                    walk(v)
        walk(lay.get("bms", {}))
        flat["_expert"] = json.dumps(lay.get("bms", {}))
        res.append(flat)
    return res


def main():
    gcc = shutil.which("gcc") or next(iter(glob.glob(
        r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
    tmp = tempfile.mkdtemp()
    exe = os.path.join(tmp, "enc.exe")
    com = os.path.join(ROOT, "bms", "common")
    env = dict(os.environ, PATH=os.path.dirname(gcc) + os.pathsep + os.environ["PATH"])
    subprocess.check_call([gcc, "-O2", "-I" + com, os.path.join(HERE, "dbc_check_enc.c"),
                           os.path.join(com, "bms_can.c"), "-o", exe, "-lm"], env=env)
    lines = subprocess.run([exe], capture_output=True, text=True, check=True, env=env).stdout.splitlines()
    rows = []
    for ln in lines:
        p = ln.split()
        rows.append((int(p[0]), bytes.fromhex(p[1]), list(map(float, p[2:]))))
    frames = [(k, "R", i, d) for k, (i, d, _) in enumerate(rows)]
    pcap = os.path.join(tmp, "enc.pcapng")
    write_pcapng(pcap, frames, 0, "can0")
    dec = decode(pcap)
    if len(dec) != len(rows):
        sys.exit(f"tshark decoded {len(dec)} of {len(rows)} frames")

    bad = 0
    for (i, d, v), s in zip(rows, dec):
        f = lambda k: float(s[k])
        ok = s.get("crc_ok") in ("1", "True", "true")
        if i == 0x101: ok &= near(f("Pack_V"), v[0], .05) and near(f("Pack_I"), v[1], .05) and near(f("Pack_T"), v[2], .05)
        elif i == 0x102: ok &= near(f("Cell_V_min"), v[0], .0005) and near(f("Cell_V_max"), v[1], .0005) and near(f("Ambient_T"), v[2], .05)
        elif i == 0x201: ok &= near(f("I_allow_dis"), v[0], .05) and near(f("I_allow_chg"), v[1], .05) and f("State") == v[2] and f("Cause") == v[3]
        elif i == 0x202: ok &= near(f("SOC"), v[0], .005) and near(f("Tcore_est"), v[1], .05) and near(f("SOH"), v[2], .25) and near(f("Psi"), v[3], .0025)
        elif i == 0x301: ok &= f("Cmd") == v[0] and f("Cmd_Value") == v[1]
        elif 0x110 <= i <= 0x114:
            k = i - 0x110
            valid, bal = int(v[4]), int(v[5])
            for j in range(4):
                g = 4 * k + j + 1
                if g > 19:
                    continue
                if valid >> j & 1:
                    ok &= near(f(f"Group{g:02d}_V"), v[j], .0005)
                ok &= f(f"Group{g:02d}_BalFb") == (bal >> j & 1)
        elif i == 0x210:
            mask = sum(int(f(f"Bal_Group{g:02d}")) << (g - 1) for g in range(1, 20))
            fl = sum(int(f(x)) << b for b, x in enumerate(("Bal_Active", "Bal_Stale", "Bal_Drive", "Bal_Blocked")))
            ok &= mask == v[0] and fl == v[1] and near(f("Cell_dV"), v[2], .00005)
        cnt = [k for k in s if k.endswith("_Counter")]
        ok &= bool(cnt) and int(f(cnt[0])) == d[6] & 15
        bad += not ok
        if not ok and bad < 5:
            print("MISMATCH", hex(i), d.hex(), v, {k: s[k] for k in s if k != "_expert"})
    print(f"{len(rows)} frames, {bad} mismatches, messages: {sorted({hex(r[0]) for r in rows})}")

    # bms.frame_bits == lojik analizörün hatta ölçtüğü bit sayısı (build/la/la_stats.csv, IFS dahil)
    la = os.path.join(ROOT, "build", "la", "la_stats.csv")
    if os.path.exists(la):
        import csv
        lr = list(csv.DictReader(open(la)))
        fr = [(k, "R", int(r["id"], 16), bytes.fromhex(r["data"])) for k, r in enumerate(lr)]
        pl = os.path.join(tmp, "la.pcapng")
        write_pcapng(pl, fr, 0, "can0")
        got = [int(s["frame_bits"]) for s in decode(pl)]
        nb = sum(g != int(r["bits"]) for g, r in zip(got, lr))
        print(f"frame_bits vs logic analyzer: {len(lr)} frames, {nb} mismatches")
        bad += nb

    # bozulmuş kopyalar: expert info üretmeli
    base = [r for r in rows if r[0] == 0x201][:6]
    t, fr, cnt = 0, [], 0
    for k, (i, d, _) in enumerate(base):
        dd = bytearray(d)
        dd[6] = (dd[6] & 0xF0) | cnt
        from can_log import crc8_j1850
        dd[7] = crc8_j1850(bytes([i & 0xFF, i >> 8]) + bytes(dd[:7]))
        if k == 2: dd[7] ^= 0x01                        # CRC bozuk
        if k != 4: cnt = (cnt + 1) & 0x0F               # 4. çerçevede sayaç donar (5. ile aynı)
        t += 300 if k == 5 else 100                     # son çerçeve 300 ms boşlukla
        fr.append((t, "T", i, bytes(dd)))
    pcap2 = os.path.join(tmp, "bad.pcapng")
    write_pcapng(pcap2, fr, 0, "can0")
    ex = [s["_expert"] for s in decode(pcap2)]
    want = {2: "bms.crc_bad", 5: "bms.cnt_bad"}
    got_ok = all(want[k] in ex[k] for k in want) and "bms.gap" in ex[5] and "bms.crc_bad" not in ex[1]
    print("fault frames:", "expert info OK" if got_ok else "expert info MISSING")
    sys.exit(1 if bad or not got_ok else 0)


if __name__ == "__main__":
    main()
