# Bench wiring sheet (HIL test bench)

Pin assignments come from the code (CLAUDE.md, `arduino/twin_emu/twin_emu.ino`, `bms/bsp/*`). Printable version: `docs/bench_wiring.pdf`.
Layout: **left = ECU (STM32), middle = ECU outputs, right = battery twin (Uno)**. Only CANH, CANL and GND cross from left to right.
Removed for simplicity (2026-11-23): **LCD** (the web panel on the tablet/PC shows everything; twin_emu detects it is missing) and **LM35** (the twin uses a fixed 25 °C; the LIMP button injects the sensor fault in software).

Wire colours: red = 5 V, black = GND, yellow = CANH, green = CANL (twisted together), other colours = signals.

## Layout (top view)

```
+--------------------------------------------------------------------+
|  ECU (left)           |  outputs        |  BATTERY TWIN (right)     |
|                       |                 |  (G) (Y) (B) (R) (W)      |
|               [TJA]   |  [relay]        |  [pot]                    |
|   NUCLEO        ||    |  [ULN2003]      |            UNO  (USB ->)  |
|   (<- USB)      ||CAN |  [LED module]   |                           |
|                 ++====+=================+=====> [MCP2515]           |
+--------------------------------------------------------------------+
```

Buttons left to right: **G** green = NORMAL, **Y** yellow = DERATE, **B** blue = LIMP, **R** red = STOP, **W** white = WATCHDOG.

## What goes where (28 wires)

| # | From | To | Block |
|---|---|---|---|
| 1 | Nucleo PA12 (CN10-12) | TJA1050 TXD | Core: CAN |
| 2 | Nucleo PA11 (CN10-14) | TJA1050 RXD | Core: CAN |
| 3 | Nucleo 5V | TJA1050 VCC | Core: CAN |
| 4 | Nucleo GND | TJA1050 GND | Core: CAN |
| 5 | TJA1050 CANH | MCP2515 module CANH | Core: CAN bus (twisted with 6) |
| 6 | TJA1050 CANL | MCP2515 module CANL | Core: CAN bus |
| 7 | Nucleo GND | Uno GND | Core: common ground |
| 8 | Uno 5V | MCP2515 VCC | Core: twin CAN |
| 9 | Uno GND | MCP2515 GND | Core: twin CAN |
| 10 | Uno D10 | MCP2515 CS | Core: twin CAN |
| 11 | **Uno D9** | MCP2515 SI | Core: twin CAN (see note: D11 is damaged on this Uno) |
| 12 | Uno D12 | MCP2515 SO | Core: twin CAN |
| 13 | Uno D13 | MCP2515 SCK | Core: twin CAN |
| 14 | Uno D2 | MCP2515 INT | Core: twin CAN |
| 15 | Nucleo D4 (PB5) | Relay IN | Contactor |
| 16 | Nucleo 5V | Relay VCC | Contactor |
| 17 | Nucleo GND | Relay GND | Contactor |
| 18 | Nucleo D7 (PA8) | ULN2003 IN1 | Headlights |
| 19 | Nucleo 5V | ULN2003 + and LED module + | Headlights |
| 20 | Nucleo GND | ULN2003 − | Headlights |
| 21 | LED module − | ULN2003 OUT1 | Headlights |
| 22 | Uno D3 | Green button (NORMAL) → GND rail | Buttons |
| 23 | Uno A2 | Yellow button (DERATE) → GND rail | Buttons |
| 24 | Uno A3 | Blue button (LIMP) → GND rail | Buttons |
| 25 | Uno D4 | Red button (STOP) → GND rail | Buttons |
| 26 | Uno D5 | White button (WATCHDOG) → GND rail | Buttons |
| 27 | Uno GND | Button GND rail | Buttons |
| 28 | Uno 5V / A0 / GND | Pot end / wiper / other end | Fault severity |

Only rows 1–14 are required; every other block is optional and the firmware runs without it.

## Notes per part

- **MCP2515 SI on D9 (2026-11-26):** the bench Uno's hardware MOSI pin (D11 = PB3) is damaged; neither D11 nor the ICSP MOSI pin drives (measured with `arduino/uno_spi_selftest`). twin_emu therefore drives the MCP2515 with software SPI: MOSI D9, SCK D13, MISO D12, CS D10 (`arduino/twin_emu/sw_spi_cfg.h`, `TW_SW_SPI 1`). Leave D11 unconnected. On a healthy Uno set `TW_SW_SPI 0` and wire SI to D11.
- **PA11/PA12:** morpho connector CN10. The pin numbers above are from memory, so check the board silkscreen. Do not add a pull-up or pull-down: TJA1050 RXD drives 5 V, and the pin is 5 V tolerant.
- **Relay (PB5):** active-low, open-drain. Do not pull it up to 3.3 V.
- **LED module:** always through the ULN2003 OUT1. Driving it straight from a GPIO with the relay on caused CAN bus-off (TEC ~250).
- **D8 (PA9, high beam):** leave it unconnected.
- **Buttons:** one leg to the pin, the other to the common GND rail. The internal pull-up is used, so no resistor is needed.
- **Pot:** fully clockwise = strongest fault (R_u ×8, 1.17 °C/s). Label it "Fault severity". Without a pot, set it over serial with `v 0-100`.
- **D0/D1 (PA2/PA3) on the Nucleo:** reserved for COM9 telemetry; connect nothing.

## CAN bus

- Connect CANH to CANH and CANL to CANL. Keep the pair short and twisted.
- Exactly **two** 120 Ω terminations: the MCP2515 **J1** jumper and the resistor on the TJA1050 module.
- With power off, measure between CANH and CANL:

| Reading | Meaning |
|---|---|
| ~60 Ω | Correct |
| ~120 Ω | One termination is missing |
| ~40 Ω | There are three; remove one |

Measured on this bench (2026-11-26): ~180 Ω at both ends. Higher than ideal, but it works on the short bench cable at 500 kbit/s.

## Power

- Each board is powered from its own USB.
- On each side, distribute 5 V from a single point with short, thick wires.
- One common GND line joins the two sides (row 7).
- Breadboard rails are often split in the middle: bridge them. A loose 5 V rail has broken the bench before.
- Fit a 100 µF capacitor next to the relay module.
- On a perfboard, mount the modules on female headers so a faulty one can be swapped.

## Check after rewiring

1. Start the EMU first (opening COM10 resets the Uno), then the ECU.
2. Run `python tools/twin_emu_run.py --dur 200`. Expect PASS, CAN errors 0, ECU NORMAL.
3. Run `python tools/out_demo.py` (needs a BMS_OUTPUTS=1 image). Expect TEC 0 while the LEDs and relay switch.
4. Run the scenarios: `python tools/demo_run.py tools/demo/modlar.txt`, then `tools/demo/butonlar.txt`.

## Faster demos (no code change)

- **Overheating:** turn the pot fully up, or send `v 100`. The pack then heats at 1.17 °C/s (default 0.5).
- **Cooling fault:** send `v 100` (R_u ×8), then load the pack fully with `m 1` and `p 100`.
- Press the cooling-fault button at least 2 min after a service restart. The cooling alarm is blanked for 120 s after ECU boot by design.

## On the day

1. Power the bench.
2. Start the gateway: `python tools/web/gateway.py COM9 --emu COM10`.
3. Reset the ECU over SWD. Opening COM10 resets the Uno, which drops the ECU to SAFE.
4. Open `https://hizlipdf.net/bitirmeprj/` on the tablet.
5. Mobile data is enough as a backup.
