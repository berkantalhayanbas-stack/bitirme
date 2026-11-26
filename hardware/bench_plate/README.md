# Bench plate (3D print)

Plain base plate for the HIL bench: one plate, two parts joined by a jigsaw edge, each part fits a 220 x 220 mm bed.
Layout follows `docs/bench_wiring.md`: ECU on the left, outputs in the middle, battery twin on the right; only the CAN pair and GND cross (cable channel).

| File | What |
|---|---|
| `bench_plate.py` | generator (all sizes at the top, in mm) |
| `bench_plate_left.stl`, `bench_plate_right.stl` | print files (3 mm plate, 4 mm corner holders, Uno posts) |
| `preview.png` | top view with module footprints |

Before printing:
1. Measure each module (outer size) with a ruler and correct `MODULES` in `bench_plate.py`. Only the Arduino Uno R3 hole positions are exact; the other modules sit between corner holders (double-sided tape).
2. Run `python hardware/bench_plate/bench_plate.py` again and check `preview.png`.

Print: PLA, 0.2 mm layers, 2-3 walls, 15-20 % infill, no supports. The Uno posts take M3 self-tapping screws (2.6 mm pilot).
The jigsaw joint is a snug fit (0.25 mm gap); glue it if it is loose.
