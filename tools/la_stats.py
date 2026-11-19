"""
Lojik analizörle (APP_LA) gerçek çerçeve uzunluğu istatistiği: hat yükü hesabını ölçümle doğrular.

Kartla art arda N yakalama alınır (la_view.capture/decode), çözülen her çerçevenin ham bit sayısı
(SOF..EOF, doldurma bitleri dahil; + 3 bit IFS) kimliğe göre toplanır. Her yakalama tek tetikle
3,855 ms sürdüğü için 0x101/0x102 çifti ya da tek bir hücre çerçevesi (0x110-0x114) düşer;
N yeterince büyükse 7 kimliğin hepsi görülür.

Kullanım (önce: fw_build.py --app 7 --flash, EMU hatta göndersin):
  python tools/la_stats.py --port COM9 -n 60
Çıktı: build/la/la_stats.csv (çerçeve başına satır), her kimliğin ilk yakalaması build/la/raw_<id>.npy
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import la_view  # noqa: E402

OUT = os.path.join(la_view.ROOT, "build", "la")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9")
    ap.add_argument("-n", type=int, default=60)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    rows, seen = [], set()
    for k in range(a.n):
        rate, x = la_view.capture(a.port)
        _, _, frames, _ = la_view.decode(rate, x)
        for f in frames:
            if f.get("error") or f["crc_rx"] != f["crc_calc"] or not f["eof_ok"]:
                continue
            rows.append(dict(cap=k, id=f"0x{f['id']:03X}", dlc=f["dlc"], stuff=f["n_stuff"],
                             bits=f["n_bits"] + 3, ack=int(f["ack"]), data=f["data"].hex()))
            if f["id"] not in seen:
                seen.add(f["id"])
                np.save(os.path.join(OUT, f"raw_{f['id']:03X}.npy"),
                        np.concatenate(([rate], x)).astype(np.int64))
        time.sleep(np.random.uniform(0.0, 0.1))   # EMU döngüsüyle senkron kalmasın
    with open(os.path.join(OUT, "la_stats.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("RESULT | ID | n | stuff min/mean/max | bits min/mean/max (incl. IFS)")
    for i in sorted({r["id"] for r in rows}):
        s = [r["stuff"] for r in rows if r["id"] == i]
        b = [r["bits"] for r in rows if r["id"] == i]
        print(f"RESULT | {i} | {len(s)} | {min(s)}/{np.mean(s):.1f}/{max(s)} | {min(b)}/{np.mean(b):.1f}/{max(b)}")
    print(f"RESULT | {len(rows)} frames, all ACKed: {all(r['ack'] for r in rows)}")


if __name__ == "__main__":
    main()
