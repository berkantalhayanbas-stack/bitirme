r"""Tezgâh tabanı (3D baskı): HIL tezgâhı üstten görünen bir EV "skateboard" şasisi olarak. Yazı yok.

    python hardware/bench_plate/bench_plate.py
    -> hardware/bench_plate/bench_plate_left.stl (ön), bench_plate_right.stl (arka), preview.png
       (iki parça da 220 x 220 mm tablaya sığar)

Araç önü solda (x = 0). Her modül gerçek araçta duracağı yerde:
  burun        : LED modülü (kısa/uzun far) + ULN2003 (far sürücüsü)
  kabin konsolu: 5 buton + pot (servis/test paneli)
  taban ortası : batarya bölgesi (kabartma çerçeve) içinde Uno + MCP2515 = paketin dijital ikizi (EMU)
  arka         : Nucleo (BMS ECU) + TJA1050, röle = ana kontaktör (paket -> arka motor)
Uzun kablo kanalı (omurga) şasenin bir kenarı boyunca ECU'dan buruna gider (far sinyalleri);
CAN çifti kısa kanaldan batarya bölgesinden ECU'ya geçer. Dört tekerlek tabanın yanına yapışık alçak bloklar.
Tasarım yalnız EKLEMELİ (taban + köşe tutucular + dikmeler + duvarlar): dilimleyici örtüşen gövdeleri birleştirir,
destek gerekmez. İki parça yapboz kenarıyla birleşir (sıkı geçme; gerekirse yapıştırıcı).
Modül ölçüleri tipik değer: ölçülürse MODULES tablosunu düzelt (mm, dış ölçü). Uno R3 delikleri kesin.
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
W, H = 300.0, 150.0          # şase tabanı (uzunluk x genişlik)
T = 3.0                      # taban kalınlığı
CH = 15.0                    # köşe pahları (burun / arka)
SPLIT = 172.0                # ön/arka ayrımı (batarya bölgesi ile ECU arası; hiçbir modül üstünden geçmez)
TEETH_Y = (70.0, 115.0)      # yapboz dişleri (bu y'lerde ayrım çizgisi yakınında parça yok)
CLR = 0.6                    # modül çevresinde boşluk
WALL, WALL_H, ARM = 1.8, 4.0, 9.0   # köşe tutucu: duvar kalınlığı, yüksekliği, kol uzunluğu
RIB_H = 3.0                  # kablo kanalı duvar yüksekliği
JIG_R = 7.0                  # yapboz dişinin yarıçapı
# tekerlekler: dingil başına (x0, x1), tabanın iki yanında; genişlik, yükseklik, tabana bindirme
WHEELS_X = [(15.0, 75.0), (215.0, 275.0)]
WHEEL_W, WHEEL_H, WHEEL_OV = 20.0, 6.0, 1.5
# batarya bölgesi: kabartma çerçeve (x0, y0, x1, y1), duvar kalınlığı, yükseklik
PACK = (100.0, 15.0, 170.0, 147.0)
PACK_WALL, PACK_H = 1.6, 1.5

# modül: ad -> (x, y sol alt köşe, genişlik (x), derinlik (y)); ölçüler TİPİK
MODULES = {
    "LED module":    (6, 45, 20.0, 60.0),      # far LED'leri (burun)
    "ULN2003":       (32, 57, 32.0, 35.0),     # far sürücüsü
    "Buttons x5":    (72, 26, 22.0, 90.0),     # 5 mod butonu, konsol boyunca (kart 90 x 22, dik konur)
    "Pot":           (72, 122, 24.0, 24.0),    # arıza şiddeti potu
    "MCP2515":       (108, 20, 44.0, 28.0),    # MCP2515 + TJA1050 (ikiz tarafı), CAN ucu sağa (ECU'ya) bakar
    "Arduino Uno":   (108, 70, 53.3, 68.6),    # Uno R3, 90° döndürülmüş: USB şase yan kenarına (+y) bakar
    "TJA1050":       (185, 30, 22.0, 12.0),    # CAN transceiver (ECU tarafı)
    "Nucleo-G431RB": (205, 55, 70.0, 82.5),    # BMS ECU; USB yan kenara (+y) bakar
    "Relay 1ch":     (225, 18, 50.0, 26.0),    # ana kontaktör (5 V röle modülü, yatık)
}
# alttan pimi çıkan kartın tutucusu daha yüksek (Nucleo'nun morpho pimleri alttan ~7 mm; kart pimlerin üstünde durur)
HOLDER_H = {"Nucleo-G431RB": 11.0}
# Uno R3 montaj delikleri, kartın sol alt köşesine göre (Arduino referans çizimi; USB sol kenarda, kart 68,6 x 53,3)
UNO_HOLES = [(14.0, 2.5), (15.3, 50.7), (66.1, 7.6), (66.1, 35.5)]
UNO_L = 68.6
POST_D, POST_HOLE, POST_H = 6.0, 2.6, 5.0      # dikme dış çap, M3 kendinden kılavuzlu vida deliği, yükseklik
# kablo kanalları: başlangıç -> bitiş (yatay), iki paralel duvar, aralık KAN_W
KAN_W = 8.0
CHANNELS = [((30, 8), (215, 8)),               # omurga: ECU -> burun (far sinyalleri)
            ((154, 36), (183, 36))]            # CAN çifti + GND: MCP2515 -> TJA1050

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
    """Aşağıdan yukarıya ayrım çizgisi: düz, diş (ön parçadan arkaya doğru), düz, diş, düz."""
    pts = [(SPLIT, 0.0)]
    for yc in TEETH_Y:
        pts.append((SPLIT, yc - JIG_R * 0.6))
        for k in range(n_seg + 1):
            a = math.pi * (-0.75 + 1.5 * k / n_seg)
            pts.append((SPLIT + JIG_R * 0.9 + JIG_R * math.cos(a), yc + JIG_R * math.sin(a)))
        pts.append((SPLIT, yc + JIG_R * 0.6))
    pts.append((SPLIT, H))
    return pts


edge = jig_edge()
GAP = 0.25                                   # geçme boşluğu: arka parçanın kenarı x'te biraz geri
left_poly = [(CH, 0.0)] + edge + [(CH, H), (0.0, H - CH), (0.0, CH)]
right_poly = [(x + GAP, y) for x, y in edge] + [(W - CH, H), (W, H - CH), (W, CH), (W - CH, 0.0)]
prism("left", left_poly, 0.0, T)
prism("right", right_poly, 0.0, T)


def part_of(x):
    return "left" if x < SPLIT - 2 else "right"


# ---------------------------------------------------------------- köşe tutucular
def corners(name, x, y, w, d):
    x0, y0, x1, y1 = x - CLR, y - CLR, x + w + CLR, y + d + CLR
    p = part_of((x0 + x1) / 2)
    z0, z1 = T, T + HOLDER_H.get(name, WALL_H)
    for cx, cy, sx, sy in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x1, y1, -1, -1), (x0, y1, 1, -1)):
        # yatay kol
        xa, xb = sorted((cx - sx * WALL, cx + sx * ARM)); ya, yb = sorted((cy - sy * WALL, cy))
        box(p, xa, ya, z0, xb, yb, z1)
        # dikey kol
        xa, xb = sorted((cx - sx * WALL, cx)); ya, yb = sorted((cy - sy * WALL, cy + sy * ARM))
        box(p, xa, ya, z0, xb, yb, z1)


for name, (x, y, w, d) in MODULES.items():
    corners(name, x, y, w, d)

# Uno dikmeleri (90° döndürülmüş: kart (bx, by) -> şase (ux + by, uy + UNO_L - bx); USB +y kenarında)
ux, uy, uw, ud = MODULES["Arduino Uno"]
POSTS = [(ux + hy, uy + UNO_L - hx) for hx, hy in UNO_HOLES]
for px, py in POSTS:
    tube(part_of(px), px, py, T, T + POST_H, POST_D / 2, POST_HOLE / 2)

# tekerlekler: tabanın iki yanında, tabana WHEEL_OV bindirerek (tabanla tek gövde basılır)
for x0, x1 in WHEELS_X:
    p_ = part_of((x0 + x1) / 2)
    box(p_, x0, -WHEEL_W, 0.0, x1, WHEEL_OV, WHEEL_H)
    box(p_, x0, H - WHEEL_OV, 0.0, x1, H + WHEEL_W, WHEEL_H)

# batarya bölgesi çerçevesi
bx0, by0, bx1, by1 = PACK
for r in ((bx0, by0, bx1, by0 + PACK_WALL), (bx0, by1 - PACK_WALL, bx1, by1),
          (bx0, by0, bx0 + PACK_WALL, by1), (bx1 - PACK_WALL, by0, bx1, by1)):
    box(part_of((r[0] + r[2]) / 2), r[0], r[1], T, r[2], r[3], T + PACK_H)

# kablo kanalları: iki paralel düşük duvar (ayrım çizgisinde 3 mm boşluk)
for (xa, ya), (xb, yb) in CHANNELS:
    for off in (-KAN_W / 2, KAN_W / 2):
        y = ya + off
        for p_, (x0, x1) in (("left", (xa, min(xb, SPLIT - 3))), ("right", (max(xa, SPLIT + 3), xb))):
            if x1 - x0 > 1:
                box(p_, x0, y - 0.8, T, x1, y + 0.8, T + RIB_H)

for p_ in ("left", "right"):
    write_stl(os.path.join(HERE, f"bench_plate_{p_}.stl"), tris[p_])

# ---------------------------------------------------------------- önizleme (üstten)
fig, ax = plt.subplots(figsize=(9.5, 6.2))
for x0, x1 in WHEELS_X:
    for y0 in (-WHEEL_W, H - WHEEL_OV):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, WHEEL_W + WHEEL_OV, fc="#3b3a37", ec="#3b3a37"))
ax.add_patch(Polygon(left_poly, closed=True, fc="#e9e8e2", ec="#52514e", lw=1))
ax.add_patch(Polygon(right_poly, closed=True, fc="#e1e0d9", ec="#52514e", lw=1))
ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fc="#fbe7c6", ec="#c98a1b", lw=1.4, alpha=0.7))
ax.text(bx0 + 3, by1 - 3, "battery zone", fontsize=7, color="#8a5a00", va="top")
for name, (x, y, w, d) in MODULES.items():
    ax.add_patch(Rectangle((x, y), w, d, fc="#cde2fb", ec="#2a78d6", lw=1))
    ax.text(x + w / 2, y + d / 2, f"{name}\n{w:g} x {d:g}", ha="center", va="center", fontsize=6.5,
            rotation=90 if d > 2.2 * w else 0)
for px, py in POSTS:
    ax.add_patch(Circle((px, py), POST_D / 2, fc="#52514e"))
for (xa, ya), (xb, yb) in CHANNELS:
    ax.add_patch(Rectangle((xa, ya - KAN_W / 2), xb - xa, KAN_W, fc="none", ec="#eb6834", ls="--", lw=1))
nx, ny, nw, nd = MODULES["Nucleo-G431RB"]
ax.annotate("USB", (nx + nw / 2, ny + nd), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7)
ax.annotate("USB", (ux + 38, uy + ud), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7)
ax.text(-4, H / 2, "front", ha="right", va="center", fontsize=8, rotation=90)
ax.text(W + 4, H / 2, "rear", ha="left", va="center", fontsize=8, rotation=90)
ax.set_xlim(-14, W + 14); ax.set_ylim(-WHEEL_W - 5, H + WHEEL_W + 5); ax.set_aspect("equal")
ax.set_title(f"Bench chassis {W:g} x {H:g} mm (+ wheels), two parts split at x = {SPLIT:g}; "
             "dashed = cable channels", fontsize=9)
ax.set_xlabel("mm"); ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "preview.png"), dpi=150)
for p_ in ("left", "right"):
    xs = [v[0] for t in tris[p_] for v in t]; ys = [v[1] for t in tris[p_] for v in t]
    print(f"{p_}: {len(tris[p_])} triangles, footprint {max(xs) - min(xs):.1f} x {max(ys) - min(ys):.1f} mm")
print("->", HERE)
