# Battery health (SOH) with persistent learning: design

Status: approved 2026-11-25, in implementation. Stress index (Ψ) and the ML tree are out of scope here and will be discussed separately.

## Goal
A "Battery Health" card on the web panel, like the iPhone one:

| Field | Meaning | Source |
|---|---|---|
| **Maximum capacity XX %** | usable capacity / nominal (12 × 4.85 Ah = 58.2 Ah) | learned on-line (new) |
| **Peak performance: normal / reduced** | internal resistance growth; reduced when SOP is cut by k_R | EKF k_R (exists) |
| **Cycle count** | equivalent full cycles = Ah throughput / (2 × 58.2 Ah) | counted (new) |

All three survive power cycles and firmware updates. A new pack starts at 100 %. The displayed capacity is nearly monotonic.

## 1. Capacity measurement: rest to rest
After a rest the cell voltage gives SOC from the OCV curve, independent of capacity. The current integrated between two rests gives the charge moved:

    Q_meas = | ∫ (I − b_I) dt | / | SOC_ocv(B) − SOC_ocv(A) |

- **Not circular:** neither end uses the EKF SOC, which itself depends on capacity.
- **Rest point:** |filtered (I − b0)| < 0.5 A for 20 min. OCV = mean pack V / 19 over the last 10 s, raw (relaxation finished); SOC from the OCV table (see "Changes").
- **Accept a measurement only if all of these hold:**
  - |ΔSOC| ≥ 0.30.
  - T in 10–40 °C at both ends.
  - Both ends sit off the flat OCV region: dOCV/dSOC ≥ threshold, taken from the polynomial.
  - No sensor or CAN fault and no SOC set command since anchor A.
  - Anchor A is younger than 24 h.
  - The result lies within 0.5–1.1 × nominal.
- **Anchor handling:** anchor A stays fixed across short rests until a valid measurement uses it. Each accepted measurement makes B the new anchor. Faults reset the anchor.
- **Update:**
  - SOH_C ← SOH_C + k · (Q_meas/Q_nom − SOH_C).
  - k = w / (n + w), with quality w = min(1, |ΔSOC| / 0.8) and n = number of accepted measurements, capped at 5 (forgetting).
  - The prior counts as n = 1, so the first measurement moves SOH_C halfway.
- **Display:** SOH_disp = SOH_C if it is lower, else min(SOH_C, SOH_disp + 1 %) per update.
- **EKF feedback:** q_ah = SOH_C × Q_nom at boot and after every update. This addresses the SOC drift on aged packs (MC aged runs: up to 4.9 %).

## 2. Persistence (flash)
- **Location:** pages 61–62 (0x0801E800–0x0801F7FF, 2 × 2 KB). The image CRC reference stays in page 63. The image CRC does not cover these pages.
- **Record (32 B, 4 double-words):**

      {magic, seq, SOH_C, SOH_disp, k_R, Ah_total, n_meas, CRC-32}

- **Writes:** append-only. On boot the valid record with the highest seq is used. When a page is full the other page is erased and used next.
  - A torn write fails its CRC, so the previous record wins.
  - 64 records/page with 10 k erase cycles gives > 10⁶ writes.
- **When to write:** only during rest (park), and only if something changed (a new measurement, or k_R / Ah moved by a threshold).
  - Page erase stalls the CPU about 22 ms. That is below the IWDG (100 ms) and is acceptable for the CAN RX FIFO at rest.
- **Reflash:** STM32_Programmer writes only the sectors in the ELF, so learning survives reflashing. A mass erase clears it.
- **New pack:** a new 0x301 command `CAN_CMD_ECU_NEW_PACK = 0x05` (value = unlock) resets the record to 100 %.
- **k_R at boot:** the EKF k_R starts from the stored value. The 120 s freeze stays, but it now starts from a learned value instead of 1.0.

## 3. Code structure
- `bms/common/bms_soh.*` (platform independent): `soh_init(record)`, `soh_step(...)` called after `bms_step`, `soh_record(...)`. It only flags "please persist"; it does no I/O.
- `bms/bsp/bsp_nvm.*`: flash ring read/append/erase (register level, FLASH->KEYR / PG / PER).
- `app_ecu.c`:
  - Loads the record at boot.
  - Writes when soh asks and the vehicle rests.
  - Sends SOH_disp on 0x202 and in the D line. Today 0x202 carries the ML tree output.
  - DX gains SOH_C, n_meas, anchor state and EFC.
- PC SIL and Python: decide whether `twin/` gets a Python reference. `bms_bal` has none; this was a precedent.

## 4. Validation
1. **Long-term SIL (PC):** twin with true SOH falling 100 → 80 % over N simulated days, each day = drive + park + charge. Reboots are simulated by re-initialising from the stored record.
   - Figure: true vs learned vs displayed SOH.
   - Metrics: final error, error after the first k measurements.
2. **Regression:** PIL/SIL refs, CAN replay and board MC 1000 must stay identical or PASS (no rests in those scenarios, so q_ah is unchanged).
3. **Board, persistence:** a record survives a power cycle and a reflash; NEW_PACK clears it; a torn write is simulated by aborting after the first double-word.
4. **Board, live learning:** twin `y 85`, then park 5 min, a high-current discharge (~15 min for ΔSOC ≥ 0.3), then park 5 min. The panel drops toward 85 % (first step halfway), and after a power cycle it starts at the learned value.

## Changes found during implementation (SIL, 2026-11-25)
- **Rest point at 20 min, raw voltage.**
  - The first version (300 s, OCV = V + V1 + V2 from the EKF) read the aged-pack rest SOC up to 0.9 points low: the EKF parks its model mismatch in V1/V2. This gave −4 % capacity at SOH 80 %.
  - 600 s with raw voltage: −0.3 points (aged τ2 ≈ 230 s).
  - 1200 s with raw voltage: remaining relaxation < 0.3 mV.
- **OCV lookup table, not the EKF polynomial.**
  - The 7th-order fit deviates −3.5 mV at SOC 0.45 and +2.9 mV at 0.9, which gave −2 % capacity bias.
  - Rest SOC now comes from the Kirkaldy GITT table, as commercial BMSs do.
  - Thesis caveat: on the bench the twin uses the same table (ideal). With a ~3 mV OCV model error the bias is about −2 %.
- **Own current offset.** The EKF b_I drifts during driving (0.8 → −1.1 A, known item A7), so rests were not detected. soh measures the offset at boot (contactor open, 1 s) and refreshes it at every rest point.
- **Persistent anchor.** The anchor (SOC, Ah since anchor) is part of the record, so a measurement spans key-off/key-on.
- **Slope threshold 4 mV/%.** The table minimum is 4.9 mV/% at SOC 0.90–0.95, which is the usual charge end.

## SIL results (`tools/soh/soh_run.py`, 100 days, true SOH 100 → 80 %, daily reboot)
| Metric | Value |
|---|---|
| Single measurement − true | bias −0.11 %, sd 0.68 % (198 accepted, 0 rejected) |
| Learned SOH − true (day ≥ 3) | mean +0.73 %, max 1.15 % (lag: ageing here is ~20× faster than real) |
| Final | 80.8 % learned vs 80.0 % true |
| EKF max SOC error at SOH 80 % | 0.89 % with learned capacity, 3.08 % with nominal |
| C vs Python (`twin/soh_capacity.py`, float32) | 321 366 steps, 0 mismatches |

Figure: `C:\bitirme\results\soh_learning.png`.

## Decisions (2026-11-25)
- Pack capacity uses the mean of the 19 groups (pack V / 19), not the weakest group.
- 0x202 SOH carries the new displayed SOH. The ML tree is discussed later, together with the stress index.
- A Python reference (`C:\bitirme\twin\soh_capacity.py`) is required, with a C/Python equivalence test.

## Open points
- Does the twin's charge mode end with CV (a full-charge anchor at SOC = 100 %)? If yes, it can be a second anchor type later.
- Bench rest and drive time is about 25–30 min per measurement. The long-term behaviour is shown in SIL.
- Limitation: twin and ECU share the same RC parameters, so the bench relaxation compensation is ideal. State this in the thesis.
