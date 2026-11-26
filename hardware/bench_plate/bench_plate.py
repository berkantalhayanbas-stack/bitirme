r"""Tezgâh tabanı (3D baskı): HIL tezgâhının modüllerini sade bir tabana yerleştirir. Yazı yok.

    python hardware/bench_plate/bench_plate.py
    -> hardware/bench_plate/bench_plate_left.stl, bench_plate_right.stl (220 x 220 mm tablaya sığar), preview.png

Yerleşim docs/bench_wiring.md ile aynı mantık: solda ECU (Nucleo + TJA1050), ortada çıkışlar (röle, ULN2003, LED),
sağda batarya ikizi (Uno + MCP2515, 5 buton, pot). Yalnız CAN çifti ve GND soldan sağa geçer (kanal).
Tasarım yalnız EKLEMELİ (taban + köşe tutucular + dikmeler + kanal duvarları): dilimleyici örtüşen gövdeleri
birleştirir, delik açmak gerekmez. İki parça yapboz kenarıyla birleşir (sıkı geçme; gerekirse yapıştırıcı).
Modül ölçüleri tipik değer: BASKIDAN ÖNCE CETVELLE ÖLÇ ve MODULES tablosunu düzelt (ölçüler mm, dış ölçü).
Uno R3 montaj delikleri kesin (Arduino çizimi); diğerleri köşe tutucuyla oturur (çift taraflı bant / vida yok).
"""
import math
import os
import struct

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- parametreler [mm]
W, H = 290.0, 165.0          # toplam taban
T = 3.0                      # taban kalınlığı
SPLIT = 145.0                # iki parçanın ayrıldığı x (her parça <= 220 mm)
CLR = 0.6                    # modül çevresinde boşluk
WALL, WALL_H, ARM = 1.8, 4.0, 9.0   # köşe tutucu: duvar kalınlığı, yüksekliği, kol uzunluğu
RIB_H = 3.0                  # kablo kanalı duvar yüksekliği
JIG_R = 7.0                  # yapboz dişinin yarıçapı

# modül: ad -> (x, y sol alt köşe, genişlik, derinlik); ölçüler TİPİK, ölç ve düzelt
MODULES = {
    "Nucleo-G431RB": (10, 55, 70.0, 82.5),     # NUCLEO-64: 70 x 82,5 mm; USB üst kenara bakar
    "TJA1050":       (88, 140, 22.0, 12.0),    # küçük CAN transceiver modülü (ECU tarafı)
    "Relay 1ch":     (88, 82, 26.0, 50.0),     # 5 V tek röle modülü
    "ULN2003":       (88, 25, 32.0, 35.0),     # sürücü kartı
    "LED module":    (122, 70, 20.0, 60.0),    # far LED'leri (8 LED çubuğu / küçük delikli kart)
    "MCP2515":       (165, 128, 44.0, 28.0),   # MCP2515 + TJA1050 modülü (ikiz tarafı), CAN ucu sola bakar
    "Arduino Uno":   (165, 62, 68.6, 53.3),    # Uno R3; USB sağ kenara bakar
    "Pot":           (225, 130, 24.0, 24.0),   # arıza şiddeti potu (küçük kart); Uno'nun USB fişinin önünde değil
    "Buttons x5":    (165, 15, 90.0, 22.0),    # 5 buton (12 mm tactile, delikli kart üstünde)
}
# Uno R3 montaj delikleri, kartın sol alt köşesine göre (Arduino referans çizimi; USB sol kenarda)
UNO_HOLES = [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5)]
POST_D, POST_HOLE, POST_H = 6.0, 2.6, 5.0      # dikme dış çap, M3 kendinden kılavuzlu vida deliği, yükseklik
# kablo kanalları: (x0, y0, x1, y1) iki paralel duvar, aralık KAN_W
KAN_W = 8.0
CHANNELS = [((113, 146), (162, 146)),          # CAN çifti + GND: TJA1050 -> MCP2515
            ((60, 45), (86, 45))]              # Nucleo -> çıkışlar (röle, ULN2003)

# ---------------------------------------------------------------- STL yardımcıları
tris = {"left": [], "right": []}


def add(part, a, b, c):
    tris[part].append((a, b, c))


def box(part, x0, y0, z0, x1, y1, z1):
    """Eksene hizalı kutu (12 üçgen, dışa bakan)."""
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    for f in ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
              (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)):
        add(part, p[f[0]], p[f[1]], p[f[2]])


def tube(part, cx, cy, z0, z1, ro, ri, n=32):
    """Delikli dikme (halka kesitli silindir)."""
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        o0 = (cx + ro * math.cos(a0), cy + ro * math.sin(a0)); o1 = (cx + ro * math.cos(a1), cy + ro * math.sin(a1))
        i0 = (cx + ri * math.cos(a0), cy + ri * math.sin(a0)); i1 = (cx + ri * math.cos(a1), cy + ri * math.sin(a1))
        add(part, (*o0, z0), (*o1, z0), (*o1, z1)); add(part, (*o0, z0), (*o1, z1), (*o0, z1))      # dış
        add(part, (*i1, z0), (*i0, z0), (*i0, z1)); add(part, (*i1, z0), (*i0, z1), (*i1, z1))      # iç
        add(part, (*o0, z1), (*o1, z1), (*i1, z1)); add(part, (*o0, z1), (*i1, z1), (*i0, z1))      # üst
        add(part, (*o1, z0), (*o0, z0), (*i0, z0)); add(part, (*o1, z0), (*i0, z0), (*i1, z0))      # alt


def area(poly):
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                     for i in range(len(poly)))


def ear_clip(poly):
    """Basit çokgen (saat yönü tersine) -> üçgen indeksleri."""
    idx = list(range(len(poly)))
    out = []

    def inside(p, a, b, c):
        d = lambda u, v, w: (u[0] - w[0]) * (v[1] - w[1]) - (v[0] - w[0]) * (u[1] - w[1])
        d1, d2, d3 = d(p, a, b), d(p, b, c), d(p, c, a)
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))
    guard = 0
    while len(idx) > 3 and guard < 100000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-9:
                continue                                           # içbükey köşe
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            out.append((i0, i1, i2)); idx.pop(k)
            break
    out.append(tuple(idx))
    return out


def prism(part, poly, z0, z1):
    """2B çokgeni z0..z1 arasında katı olarak çıkar (taban parçası)."""
    if area(poly) < 0:
        poly = poly[::-1]
    for a, b, c in ear_clip(poly):
        add(part, (*poly[a], z1), (*poly[b], z1), (*poly[c], z1))
        add(part, (*poly[c], z0), (*poly[b], z0), (*poly[a], z0))
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        add(part, (*p, z0), (*q, z0), (*q, z1)); add(part, (*p, z0), (*q, z1), (*p, z1))


def write_stl(path, tl):
    with open(path, "wb") as f:
        f.write(b"bench_plate".ljust(80, b" "))
        f.write(struct.pack("<I", len(tl)))
        for a, b, c in tl:
            u = [b[i] - a[i] for i in range(3)]; v = [c[i] - a[i] for i in range(3)]
            n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            L = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(struct.pack("<12fH", *(x / L for x in n), *a, *b, *c, 0))


# ---------------------------------------------------------------- yapboz kenarı (x = SPLIT, iki diş)
def jig_edge(n_seg=16):
    """Aşağıdan yukarıya ayrım çizgisi: düz, diş (sağa doğru), düz, diş, düz."""
    pts = [(SPLIT, 0.0)]
    for yc in (H * 0.3, H * 0.7):
        pts.append((SPLIT, yc - JIG_R * 0.6))
        # boyun + yuvarlak baş: sola parçadan sağa çıkıntı
        for k in range(n_seg + 1):
            a = math.pi * (-0.75 + 1.5 * k / n_seg)
            pts.append((SPLIT + JIG_R * 0.9 + JIG_R * math.cos(a), yc + JIG_R * math.sin(a)))
        pts.append((SPLIT, yc + JIG_R * 0.6))
    pts.append((SPLIT, H))
    return pts


edge = jig_edge()
GAP = 0.25                                   # geçme boşluğu: sağ parçanın kenarı x'te biraz geri
left_poly = [(0.0, 0.0)] + edge + [(0.0, H)]
right_poly = [(x + GAP, y) for x, y in edge] + [(W, H), (W, 0.0)]
prism("left", left_poly, 0.0, T)
prism("right", right_poly, 0.0, T)


def part_of(x):
    return "left" if x < SPLIT - 2 else "right"


# ---------------------------------------------------------------- köşe tutucular
def corners(name, x, y, w, d):
    x0, y0, x1, y1 = x - CLR, y - CLR, x + w + CLR, y + d + CLR
    p = part_of((x0 + x1) / 2)
    z0, z1 = T, T + WALL_H
    for cx, cy, sx, sy in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x1, y1, -1, -1), (x0, y1, 1, -1)):
        # yatay kol
        xa, xb = sorted((cx - sx * WALL, cx + sx * ARM)); ya, yb = sorted((cy - sy * WALL, cy))
        box(p, xa, ya, z0, xb, yb, z1)
        # dikey kol
        xa, xb = sorted((cx - sx * WALL, cx)); ya, yb = sorted((cy - sy * WALL, cy + sy * ARM))
        box(p, xa, ya, z0, xb, yb, z1)


for name, (x, y, w, d) in MODULES.items():
    corners(name, x, y, w, d)

# Uno dikmeleri (USB sağa bakacak biçimde 180° döndürülmüş: delikler kartın sağ üst köşesine göre ayna)
ux, uy, uw, ud = MODULES["Arduino Uno"]
for hx, hy in UNO_HOLES:
    tube("right", ux + uw - hx, uy + ud - hy, T, T + POST_H, POST_D / 2, POST_HOLE / 2)

# kablo kanalları: iki paralel düşük duvar
for (xa, ya), (xb, yb) in CHANNELS:
    for off in (-KAN_W / 2, KAN_W / 2):
        y = ya + off
        for p_, (x0, x1) in (("left", (xa, min(xb, SPLIT - 3))), ("right", (max(xa, SPLIT + 3), xb))):
            if x1 - x0 > 1:
                box(p_, x0, y - 0.8, T, x1, y + 0.8, T + RIB_H)

for p_ in ("left", "right"):
    write_stl(os.path.join(HERE, f"bench_plate_{p_}.stl"), tris[p_])

# ---------------------------------------------------------------- önizleme (üstten)
fig, ax = plt.subplots(figsize=(9, 5.6))
ax.add_patch(Polygon(left_poly, closed=True, fc="#e9e8e2", ec="#52514e", lw=1))
ax.add_patch(Polygon(right_poly, closed=True, fc="#e1e0d9", ec="#52514e", lw=1))
for name, (x, y, w, d) in MODULES.items():
    ax.add_patch(Rectangle((x, y), w, d, fc="#cde2fb", ec="#2a78d6", lw=1))
    ax.text(x + w / 2, y + d / 2, f"{name}\n{w:g} x {d:g}", ha="center", va="center", fontsize=7)
for hx, hy in UNO_HOLES:
    ax.add_patch(Circle((ux + uw - hx, uy + ud - hy), POST_D / 2, fc="#52514e"))
for (xa, ya), (xb, yb) in CHANNELS:
    ax.add_patch(Rectangle((xa, ya - KAN_W / 2), xb - xa, KAN_W, fc="none", ec="#eb6834", ls="--", lw=1))
ax.annotate("USB", (MODULES["Nucleo-G431RB"][0] + 35, MODULES["Nucleo-G431RB"][1] + 82.5), xytext=(0, 6),
            textcoords="offset points", ha="center", fontsize=7)
ax.annotate("USB", (ux + uw, uy + ud / 2), xytext=(8, 0), textcoords="offset points", va="center", fontsize=7)
ax.set_xlim(-5, W + 10); ax.set_ylim(-5, H + 10); ax.set_aspect("equal")
ax.set_title(f"Bench plate {W:g} x {H:g} mm, two parts (split at x = {SPLIT:g}); dashed = cable channels", fontsize=9)
ax.set_xlabel("mm"); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "preview.png"), dpi=150)
for p_ in ("left", "right"):
    xs = [v[0] for t in tris[p_] for v in t]; ys = [v[1] for t in tris[p_] for v in t]
    print(f"{p_}: {len(tris[p_])} triangles, footprint {max(xs) - min(xs):.1f} x {max(ys) - min(ys):.1f} mm")
print("->", HERE)
