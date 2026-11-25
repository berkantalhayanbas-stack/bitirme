r"""E2E gösterimi: heat SIL kaydının (build/can_sil_heat.log) arıza enjekteli kopyası -> build/can_fault_demo.pcapng.
Wireshark + bms_dbc.lua ile Analyze > Expert Information'da görünür (kırmızı hata / sarı uyarı).
  30,0-30,5 s  0x101 CRC baytı bozuk                      (EMU_CRC gibi)        -> 5 x CRC-8 mismatch
  40,0-41,0 s  0x102 sayacı donuk, CRC geçerli            (EMU_FREEZE gibi)     -> sayaç adımı 0 reddi
  50,0-50,5 s  0x101 gönderilmez                          (EMU_SILENT gibi)     -> boşluk > 1,5 x periyot

    python tools/can/can_log.py --sil heat --dur 300 && python tools/can/fault_demo.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
from can_log import crc8_j1850, write_pcapng  # noqa: E402

fr = []
for ln in open(os.path.join(ROOT, "build", "can_sil_heat.log")):
    m = re.match(r"\((\d+)\.(\d+)\) \w+ (\w+)#(\w+)", ln)
    t = int(m[1]) * 1000 + int(m[2]) // 1000
    i = int(m[3], 16)
    d = bytearray.fromhex(m[4])
    if i == 0x101 and 30000 <= t < 30500:
        d[7] ^= 0x5A
    if i == 0x102 and 40000 <= t < 41000:
        d[6] = (d[6] & 0xF0) | 0x7
        d[7] = crc8_j1850(bytes([i & 0xFF, i >> 8]) + bytes(d[:7]))
    if i == 0x101 and 50000 <= t < 50500:
        continue
    fr.append((t, "T" if i in (0x201, 0x202, 0x210) else "R", i, bytes(d)))
out = os.path.join(ROOT, "build", "can_fault_demo.pcapng")
write_pcapng(out, fr, 0, "can0")
print(f"{len(fr)} frames -> {out}")
