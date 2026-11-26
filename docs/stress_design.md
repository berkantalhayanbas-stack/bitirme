# Battery stress index Ψ (redesign): design

Status: IMPLEMENTED 2026-11-26 (Python, MATLAB, C; SIL, Monte Carlo). Draft of 2026-11-25. It replaces the "max + κ" aggregation in `docs/stress_criteria_review.md` and in the stress-index decisions of 2026-11-21/22. Deviations from the draft are listed in "## 5. As implemented" at the end.

## Goal
Ψ should behave like **the sum of all bad conditions** acting on the battery. Several mild conditions add up, a single severe one dominates, and the result never exceeds 1. It covers protection (thermal), ageing and health, and the panel shows which condition drives it.

## 1. Aggregation: probabilistic OR

    Ψ = 1 − Π (1 − Ψ_i),   Ψ_i ∈ [0, 1]

- For small values Ψ ≈ Σ Ψ_i. With three conditions at 0.2 each, Ψ = 0.49 (today's max + 0.25·rest gives 0.30; a plain sum gives 0.60).
- One Ψ_i = 1 gives Ψ = 1. Ψ is monotonic in every Ψ_i and has no tuning constant (κ is removed).
- Reading: each condition takes its share of whatever "health margin" is left.
- The dominant criterion is still reported (largest Ψ_i), for explainability.

## 2. Sub-indices

| id | Condition | Signal | 0 → 1 | Limits discharge | Limits charge | Status |
|---|---|---|---|---|---|---|
| Ψ_T | Core temperature | Tc_hat | 45 → 55 °C | ✓ | ✓ | exists |
| Ψ_Tdot | Temperature rise rate | filtered dT/dt | 3 → 10 °C/min | ✓ | ✓ | exists |
| Ψ_C | Cooling degraded | R_u alarm | alarm (cap 0.3) | ✓ | ✓ | exists |
| **Ψ_A** | **Ageing rate** | hybrid model, §3 | rate ratio 1× → 8× (log) | — | ✓ (floor 0.3C) | new (ML) |
| Ψ_H | Health | capacity SOH (`soh_c`) and k_R | SOH 90 → 70 %, k_R 1.2 → 1.3 (max of the two) | — | — | display (was Ψ_H on k_R only) |
| Ψ_B | Cell imbalance | V_max − V_min at \|I\| < 2 A | 50 → 150 mV | — | — | new, display |

- **Discharge limit:** I_allow_dis = min(I_ds · (1 − Ψ_P), I_SOP, …), with Ψ_P = OR(Ψ_T, Ψ_Tdot, Ψ_C).
- **Charge limit:** I_allow_chg = min(max(I_chg,ds · (1 − Ψ_PA), I_floor), …), with Ψ_PA = OR(Ψ_T, Ψ_Tdot, Ψ_C, Ψ_A).
  - I_floor ≈ 0.3C is applied to Ψ_A only. Thermal terms can still reduce charging to zero through LIMP/SAFE.
- **Panel:** "Battery stress" = Ψ = OR of all six, with a per-condition breakdown.
- **Hard protections are unchanged and deterministic:** LIMP/SAFE, datasheet limits, SOP and voltage margins. Ψ is the "gentle" layer below them.
- **Like real EVs:** charge current is limited in the cold (Li plating), when hot and when nearly full, but driving power is never cut for ageing. The difference: production cars use a static map from the cell maker, while here the limit comes from an ageing model learned from measured data.
- Ψ_B threshold sits above the twin's nominal spread (18–46 mV incl. the weak group); balancing remains the actuator.
- Already covered inside Ψ_A, so no separate criteria: high/low SOC, cold, depth of discharge.
- Sensor/CAN integrity already drives LIMP/SAFE and the panel's integrity view, so it is not added.

## 3. Ψ_A: ageing rate from real data (hybrid physics + ML)

**Data:** Kirkaldy et al. (2024), LG M50T, 40 cells, 12 cycling conditions:
- T 10/25/40 °C
- Mean SOC 0.15/0.5/0.775/0.925
- DoD 0.15/0.3/1

Expt 4 (drive cycle) is held out for validation, as in `veri/aging_calibration.py`.

**Model structure** (existing calibration): fade = a · (Σ S · dEFC)^z + calendar(V, T, t). S(T, m, dod) is the stress factor per equivalent full cycle; S = 1 at 25 °C, m 0.5, dod 1.

**Leave-one-condition-out RMSE of the capacity-fade trajectories** (`C:\bitirme\ml\06_stress_loco.py`, 2026-11-25):

| Model for S | Mean | Worst |
|---|---|---|
| Physics (Millner-type, `aging_stress.m`) | 3.78 % | 6.32 % |
| Pure ML, regression tree (depth 3) | 5.23 % | 12.92 % |
| Pure ML, gradient boosting | 5.64 % | 12.30 % |
| **Hybrid: physics S × exp(tree residual, depth 2)** | **3.64 %** | 6.77 % |

Pure ML overfits 11 training conditions and cannot extrapolate. The hybrid keeps the physics trends and lets the data correct them. It is the deployed model.

**Data finding:** for this NMC811/SiOx cell, cycling at 10 °C ages faster than at 40 °C (Li plating). So cold charging gets the highest Ψ_A, which matches real-EV practice.

**Online use (ECU, every 0.1 s):**
- Damage rate now: r = S(T, m, dod) · |I| / (2Q) + calendar rate(V, T).
  - m = running mean SOC of the current micro-cycle.
  - dod = SOC swing since the last charge/discharge reversal (a simple half-cycle rule; not full rainflow).
- Reference rate r_ref = the same at 25 °C, m 0.5, dod 1, 0.5C.
- R = r / r_ref, then Ψ_A = clamp(log2(R) / 3, 0, 1): 1× (normal) → 0, 8× → 1.
- When the pack rests, only the calendar term acts. Parking hot and full therefore raises Ψ_A, which supports an iPhone-style "optimised charging" hint on the panel ("limit charge to 80 %").

**Cause → effect link to SOH:** the accumulated damage Σ S · dEFC (stored in the flash record) gives the model-predicted capacity. The panel can show "predicted X % vs measured Y %", where measured is the capacity SOH from `docs/soh_design.md`.

**Codegen:** a new `ml/07_stress_model.py` trains the hybrid on all 12 conditions and generates:
- C `bms/common/stress_tree.c` (if-else plus physics terms)
- Python `twin/stress_model.py`
- MATLAB `simulink/stress_model.m`

The same pipeline as the old `soh_tree`. The old `soh_tree` is no longer called (0x202 carries capacity SOH since b567374) and is removed from the firmware; the thesis describes it as tried and limited by inputs that were not available online.

## 4. Implementation order
1. `ml/07_stress_model.py`: train, physics monotonicity checks, export C/Python/MATLAB, and an equivalence test of the three.
2. Python `twin/decision.py`: OR aggregation, Ψ_A / Ψ_H / Ψ_B, charge limit with floor; unit tests in `test_decision_unit.py`.
3. MATLAB `decision_logic.m`, and a Python/MATLAB equivalence check.
4. C `bms_algo.c` (`decision_step`, new inputs: soh_c, ΔV, accumulated damage) plus `bms_config.h` constants (`DEC_PSI_*`, `STRESS_*`).
5. Regenerate SIL references (`sil_ref.py`) and PIL inputs if the opening behaviour changes. Run PIL 3/3 on the board.
6. Monte Carlo 1000 runs on PC and board. Key claims:
   - No false derating in nominal runs (Ψ_P = 0).
   - Aged runs raise Ψ_H.
   - Charge in the cold raises Ψ_A and limits charging.
   - All existing verdicts still pass.
7. Telemetry: DX gains Ψ_A, Ψ_B, R and damage. Gateway, server columns, panel breakdown and an "optimised charging" hint.
8. Bench scenarios with twin_emu: cold charge (`a 5` + `m 2`), hot full park (`h 1` + `s 95` + `m 3`), imbalance; then thesis figures.

## Open points
- Ψ_A thresholds (1× → 8×) and the 0.3C floor are assumptions. Tune them with Monte Carlo and flag them as assumed in the thesis.
- C-rate is not varied in the Kirkaldy cycling data; the model's C-rate dependence is only through throughput (|I|/2Q).
- The calendar term is literature (Schmalstieg 2014, NMC) and not calibrated on M50T.

## 5. As implemented (2026-11-26)
- **Residual model:** the depth-2 residual tree splits only on temperature (its three leaves are the three temperature groups). It would make S jump by 2.3× at 22.7 °C online, so the deployed residual is the same correction, interpolated linearly between the group mean temperatures and held constant outside them. Its LOCO RMSE is 3.52 % mean and 6.68 % worst, against 3.64 / 6.77 % for the tree and 3.78 / 6.32 % for physics alone (`C:\bitirme\ml\stress_model_loco.csv`).
- **Ψ_A has two terms, combined with OR:**
  - **Cycle term:** R_cyc = S(T, m, dod)/S_ref · |I|/0.5C, filtered with τ = 60 s. It is the only term that limits charging, through (1 − Ψ_cyc) with a 0.3C floor.
  - **Calendar term:** R_cal = alpha(OCV(SOC), T)/alpha(OCV(0.5), 25 °C). It starts at 2× (so 2× gives 0 and 16× gives 1) and is display only, because limiting current does not slow calendar ageing.
  - **Half-cycle rule:** dod is the larger of the current swing and the last completed half cycle.
- **Finding, cold charge:** the datasheet already caps charging at 0.3C below 23 °C, so the floor means Ψ_A cannot reduce cold charging further. In the data, a shallow cycle at 10 °C ages the cell *less* per EFC than a full cycle at 25 °C (S_obs 0.94–2.7 vs 2.76). Only full-depth cold cycling is worse (7.2). A 0.3C full-depth charge at 15 °C gives R_cyc 0.48, against 0.39 at 26 °C. The Kirkaldy data do not vary C-rate, so plating at high C-rate is not represented. This is a thesis limitation.
- **Ψ_H:** the larger of the capacity-SOH term (90 → 70 %) and the k_R term (1.2 → 1.3); the 0.3 cap is removed. Display only.
- **Ψ_B:** 50 → 150 mV, updated only when |I| < 2 A (filter τ 10 s). Display only.
- **Cause bits:** 4096 (Ψ_A) and 8192 (Ψ_B) are informational, like 1024, and never set DERATE.
- **Verification:**
  - Python unit tests 23/23 (U17–U23).
  - MATLAB = Python on 34 830 samples (max difference 2.2e-16); Simulink FMEA B1–B8 PASS.
  - C = Python: 0 state/cause/dominant mismatches; current within 0.5 mA (float32).
  - Closed-loop S1–S9: same states; S2 core peak 48.3 → 47.3 °C, since OR derates slightly harder than max + κ.
  - PC Monte Carlo 1000/1000 PASS.
