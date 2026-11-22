r"""bms.dbc == bms/common/bms_can.c kontrolü: C kodlayıcısı 2000 x 7 rastgele çerçeve üretir, cantools
DBC ile çözer (fiziksel değer yarım LSB içinde, durum/neden/bit alanları birebir) ve DBC ile yeniden
kodlar (CRC hariç bayt bayt aynı). bms_can.* değişirse DBC'yi de güncelle ve bunu koştur.

    pip install cantools
    python tools/can/dbc_check.py
"""
import glob, os, shutil, subprocess, sys, tempfile
import cantools

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
gcc = shutil.which("gcc") or next(iter(glob.glob(
    r"C:\ProgramData\MATLAB\SupportPackages\*\3P.instrset\mingw_w64.instrset\bin\gcc.exe")), None)
if not gcc:
    sys.exit("gcc not found")
exe = os.path.join(tempfile.mkdtemp(), "dbc_check_enc.exe")
common = os.path.join(ROOT, "bms", "common")
subprocess.check_call([gcc, "-O2", "-I" + common, os.path.join(HERE, "dbc_check_enc.c"),
                       os.path.join(common, "bms_can.c"), "-o", exe, "-lm"])
lines = subprocess.run([exe], capture_output=True, text=True, check=True).stdout.splitlines()
db = cantools.database.load_file(os.path.join(HERE, "bms.dbc"))
bad = 0; n = 0
def near(a, b, tol): return abs(a - b) <= tol * 1.001 + 1e-6   # yarım LSB (float yuvarlama sınırı)
for line in lines:
    p = line.split(); i = int(p[0]); d = bytes.fromhex(p[1]); v = list(map(float, p[2:]))
    m = db.get_message_by_frame_id(i); s = m.decode(d, decode_choices=False, scaling=True)
    ok = True
    if i == 0x101: ok = near(s['Pack_V'], v[0], .05) and near(s['Pack_I'], v[1], .05) and near(s['Pack_T'], v[2], .05)
    elif i == 0x102: ok = near(s['Cell_V_min'], v[0], .0005) and near(s['Cell_V_max'], v[1], .0005) and near(s['Ambient_T'], v[2], .05)
    elif i == 0x201: ok = near(s['I_allow_dis'], v[0], .05) and near(s['I_allow_chg'], v[1], .05) and s['State'] == v[2] and s['Cause'] == v[3]
    elif i == 0x202: ok = near(s['SOC'], v[0], .005) and near(s['Tcore_est'], v[1], .05) and near(s['SOH'], v[2], .25) and near(s['Psi'], v[3], .0025)
    elif i == 0x301: ok = s['Cmd'] == v[0] and s['Cmd_Value'] == v[1]
    elif 0x110 <= i <= 0x114:
        k = i - 0x110; valid, bal = int(v[4]), int(v[5])
        for j in range(4):
            g = 4 * k + j + 1
            if g > 19: ok &= s[f'Cells{k}_Spare'] == 4095; continue
            raw = round((s[f'Group{g:02d}_V'] - 1.0) / 0.001)
            ok &= (raw == 4095) if not (valid >> j & 1) else near(s[f'Group{g:02d}_V'], v[j], .0005)
            ok &= s[f'Group{g:02d}_BalFb'] == (bal >> j & 1)
    elif i == 0x210:
        mask = sum(int(s[f'Bal_Group{g:02d}']) << (g - 1) for g in range(1, 20))
        fl = sum(int(s[x]) << b for b, x in enumerate(('Bal_Active', 'Bal_Stale', 'Bal_Drive', 'Bal_Blocked')))
        ok = mask == v[0] and fl == v[1] and near(s['Cell_dV'], v[2], .00005)
    ok &= s[[x.name for x in m.signals if x.name.endswith('_Counter')][0]] == d[6] & 15
    # ters yön: DBC ile yeniden kodla, CRC hariç aynı bayt
    raw = m.decode(d, decode_choices=False, scaling=False)
    dd = bytearray(d)
    if i == 0x114: dd[6] &= 0x7F          # 20. yuva yok: dengeleme biti (bit 55) tanımsız
    ok &= m.encode(raw, scaling=False, strict=False)[:7] == bytes(dd[:7])
    n += 1; bad += not ok
    if not ok and bad < 5: print('MISMATCH', line.strip(), s)
print(f'{n} frames, {bad} mismatches, messages covered:', sorted({hex(int(l.split()[0])) for l in lines}))
sys.exit(1 if bad else 0)
