# Bench chassis (3D print)

The HIL bench as a top view of an EV "skateboard" chassis: every module sits where it would sit in a real car.
Front is on the left. No text on the part; the shapes tell the story.

| Zone | Modules | Role |
|---|---|---|
| Nose | LED module, ULN2003 | low/high beam and its driver |
| Cabin console | 5 buttons, pot | service/test panel (mode buttons, fault severity) |
| Battery zone (raised frame) | Arduino Uno, MCP2515 | digital twin of the 19S12P pack (cell monitoring, EMU) |
| Rear | Nucleo-G431RB, TJA1050, relay | BMS ECU and main contactor (pack -> rear motor) |

A long cable channel (spine) runs along one side from the ECU to the nose (headlight signals); the CAN pair crosses
from the battery zone to the ECU in a short channel. Four low wheel blocks are fused to the sides.

| File | What |
|---|---|
| `bench_plate.py` | generator (all sizes at the top, in mm) |
| `bench_plate_left.stl` | front part, 185 x 190 mm incl. wheels |
| `bench_plate_right.stl` | rear part, 128 x 190 mm incl. wheels |
| `preview.png` | top view with module footprints |

Module sizes are typical values; only the Arduino Uno R3 hole positions are exact. If a module differs,
correct `MODULES` in `bench_plate.py`, run `python hardware/bench_plate/bench_plate.py` and check `preview.png`.
The Nucleo holders are 11 mm tall because the board stands on its bottom morpho pins.

Print: PLA, 0.2 mm layers, 2-3 walls, 15-20 % infill, no supports, flat side down. The Uno posts take M3
self-tapping screws (2.6 mm pilot); the other modules sit between corner holders (double-sided tape).
The jigsaw joint is a snug fit (0.25 mm gap); glue it if it is loose.
