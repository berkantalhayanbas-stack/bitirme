# Stress index Ψ: literature review of candidate criteria

Date: 2026-11-21. Scope: criteria an automotive BMS can compute online, to decide what to add to the multi-criteria stress index Ψ (`bms/common/bms_algo.c` `decision_step`, `DEC_*` in `bms_config.h`, `C:\bitirme\twin\decision.py`). No code was changed.

Cell: LG INR21700-M50T (NMC811 / graphite-SiOx), pack 19S12P. Datasheet limits used by the ECU: charge 0.3C standard / 0.7C max (3.395 A), discharge 1.5C max (7.275 A), 4.20 V / 2.50 V, charge 0-45 °C, discharge -20-60 °C (`DEC_T_CHG`, `DEC_T_DIS` tables).

> **Citation status.** DOIs marked † were checked online in this session. The others are cited from memory: check each one before it goes into the thesis bibliography.

---

## 1. Bottom line

1. **The bench scenarios the user flagged (sensor cut, line cut, watchdog) are not battery stress.** They are a loss of *information* or *control*, and the cell is not physically loaded more. No literature criterion of cell stress fires for them, so adding one would make Ψ less meaningful. The real dashboard problem is that a criterion whose input is invalid currently shows as **0 ("no stress")** when it should show as **unknown**. Fix this with a separate display layer (Section 4, item 4). It is cheap and answers the complaint directly.
2. **Ψ is direction-blind today.** `tgt_dis = i_ds_dis·(1−Ψ)` and `tgt_chg = i_ds_chg·(1−Ψ)` use the same Ψ. The two best-supported new criteria are one-directional: lithium plating and high voltage limit only **charge**, and low SOC limits only **discharge**. They need Ψ to be split into Ψ_chg and Ψ_dis (Section 5). No CAN change is needed, because 0x201 already carries separate charge and discharge limits.
3. Several real stresses are already limited, but **invisibly**, through `min()` terms (voltage-headroom SOP, datasheet temperature table). They show up only as cause bits. Turning them into explicit sub-indices makes the dashboard explain *why* the current is limited. The `min()` terms stay as hard guards.
4. Ageing-only stresses (high-SOC dwell, throughput, depth of discharge) cannot be reduced by lowering the current in the moment. They belong on the dashboard as an **ageing-rate gauge**, not in Ψ.

### Ranked shortlist

| Rank | Criterion | Enters Ψ? | Direction | Fires in which bench test |
|---|---|---|---|---|
| 1 | **Ψ_V: voltage-window stress** (group V_max near 4.2 V, V_min near the low knee) | yes, cap 0.3 | chg (high V) / dis (low V) | A2 short (fast charge to full, SOC 88 %), long drive to low SOC |
| 2 | **Ψ_P: lithium-plating risk** (charge C-rate vs plating-safe C-rate at Tc and SOC) | yes, cap 0.5 | chg only | A2 short, cold-day regen/charge (`t <°C>` low + mode 2) |
| 3 | **A: ageing-rate gauge** (relative degradation rate from T, SOC, C-rate; plus EFC/Ah counter) | **no**, display only | – | every scenario: shows parking at high SOC, hot day, hard driving |
| 4 | **Ω: monitoring integrity** (which inputs are invalid; affected Ψ_i shown grey "unknown") | **no**, display only | – | A3 sensor cut, D4 line cut, D5 watchdog, CRC/frozen counter |
| 5 | **Ψ_SD: abnormal self-discharge** (group deviation drift at rest; internal-short precursor) | warning first, LIMP later | both | none yet: needs a new EMU fault |

Criteria dropped on 2026-11-21 (I²t, static ΔV, σ_SOC) stay dropped. Section 6 gives the literature reason in each case.

---

## 2. What the current Ψ already covers

| id | signal | mechanism | literature |
|---|---|---|---|
| Ψ_T | Tc_hat 43→55 °C | Arrhenius acceleration of SEI growth and cathode side reactions; thermal-runaway margin | Waldmann et al. 2014; Feng et al. 2018 |
| Ψ_C | cooling alarm (R_u estimate) | loss of heat rejection → higher T for the same load | thermal-management design practice |
| Ψ_Tdot | filtered dT/dt 3→10 °C/min | early sign of abnormal heat generation, ahead of the lagging Tc estimate | Feng et al. 2018 (self-heating precursors) |
| Ψ_H | EKF k_R 1.2→1.3 | resistance growth = more heat per A, earlier voltage limits, ageing marker | Birkl et al. 2017; Edge et al. 2021 |

All four are **thermal or health** criteria. Nothing in Ψ describes **electrochemical** stress, i.e. where the electrode potentials sit. That is the main gap the literature points to.

---

## 3. Candidate criteria in detail

Each entry gives: mechanism and risk / online signal / M50T thresholds / role.

### 3.1 Ψ_V: voltage-window stress (high and low SOC edge)

**Mechanism.**
- *High voltage, high SOC (charge).* Ni-rich NMC goes through the H2→H3 phase transition near the top of charge (about 4.1-4.2 V full-cell). The large, abrupt c-axis contraction cracks particles and exposes fresh surface to the electrolyte. Above about 80 % SOC, lattice oxygen release and electrolyte oxidation also accelerate (Ryu et al. 2018; Jung et al. 2017). At the same time the anode potential is closest to 0 V vs Li, which raises the plating risk (see 3.2).
- *Low voltage, low SOC (discharge).* In the M50T the SiOx is mainly delithiated at low SOC. Cycling there puts the large volume change of silicon to work. Kirkaldy et al. 2022† measured about 80 % loss of active silicon after about 4 kAh at 0-30 % SOC and 40 °C on **this exact cell**, against about 10 % graphite loss. They concluded that silicon loss is worst at low SOC and high temperature. Deep over-discharge (< 2 V) dissolves the Cu collector and can create internal shorts (Guo et al. 2016), but this is already covered by `DEC_V_SAFE_LO` = 2.50 V.

**Online signal.** Group voltages from 0x110-0x114 (already filtered for balancing): `V_max` when charging, `V_min` when discharging, and the EKF SOC as a cross-check. These are the same inputs as the existing SOP term, so the criterion costs a few float operations.

**Proposed thresholds (M50T).**

| direction | on | full | cap | note |
|---|---|---|---|---|
| charge | V_max 4.10 V (≈ 90 % SOC at rest) | 4.18 V (`DEC_V_LIM_CHG`) | 0.3 | H2-H3 region |
| discharge | V_min 3.30 V (≈ 10-12 % SOC at rest) | 3.00 V | 0.3 | Si-active region; above `DEC_V_LIM_DIS` 2.90 V |

Under load, the measured V includes I·R. Using `V_max − |I|·r0` (charge) and `V_min + I·r0` (discharge), i.e. an OCV-like value, keeps the criterion about electrode state rather than about the instantaneous IR drop. The SOP term already computes these quantities.

**Role.** Enters Ψ, direction-specific. This has to wait for the Ψ split (Section 5): with today's shared Ψ, a high-SOC charge stress would also cut driving current, which is wrong. The existing SOP `min()` stays as the hard voltage guard. Ψ_V makes its approach visible and gradual.

**Thesis value.** It is the first electrochemical criterion. It fires in the A2-short button test (fast charge to a full battery), which today shows no Ψ at all.

### 3.2 Ψ_P: lithium-plating risk during charge (including regen)

**Mechanism.** Plating happens when the graphite surface potential drops below 0 V vs Li/Li⁺. That is favoured by low temperature (slow solid diffusion and charge transfer), high charge C-rate and high SOC. Plated Li consumes cyclable lithium, can form dendrites, and lowers the thermal-runaway onset temperature. It is the dominant fast-charge and cold-charge ageing mechanism (Waldmann et al. 2018; Tomaszewska et al. 2019; Yang et al. 2018). Below about 25 °C, ageing rate *increases* as temperature falls, the reverse of the Arrhenius trend above 25 °C (Waldmann et al. 2014). Regen braking in winter is the typical automotive case.

**Online signal.** Charge current per cell `c = −I/(N_P·Q)` (C-rate), core temperature `Tc_hat` (the anode sits at core temperature, not surface temperature), and EKF SOC.

Risk ratio `r = c / C_safe(Tc, SOC)`, where
- `C_safe(T)` = the datasheet charge envelope at the core temperature. The ECU already holds it in `DEC_T_CHG`/`DEC_I_CHG` (0 at 0 °C, 0.3C at 2-23 °C, 0.7C at 25-43 °C). LG's rating is in effect the manufacturer's plating-safe limit.
- the SOC factor `k_SOC = 1` below 70 % SOC, falling linearly to 0.5 at 95 %. This reflects the lower anode potential at high SOC (Waldmann et al. 2018; Tomaszewska et al. 2019). The 70 %/0.5 values are **assumptions** to be tuned. Kirkaldy et al. 2024 charged at 0.3C to 4.2 V without reporting plating at 10 °C, which is consistent with the datasheet line.

**Thresholds.** Ψ_P = 0 at r = 0.7, full at r = 1.0, cap 0.5.

**Role.** Enters Ψ_chg only. It differs from the existing datasheet `min()` in three ways: it uses Tc instead of Ts, it adds the SOC factor, and it starts *before* the hard limit, so the dashboard shows "charging too fast for this temperature/SOC".

**Caution.** Our twin has no plating model, so validation can only show that the criterion fires where it should (cold and/or high SOC charging). It cannot show prevented degradation. State this as a limitation.

### 3.3 A: ageing-rate gauge (display only)

**Mechanism.** Semi-empirical life models write the degradation rate as a product of stress factors: temperature (Arrhenius), SOC (calendar; anode potential), C-rate and depth of discharge (cycle) (Schmalstieg et al. 2014; Xu et al. 2018; Smith et al. 2017). Calendar fade grows strongly with storage SOC, in steps that follow the graphite potential plateaus (Keil et al. 2016). It also grows with T. Across chemistries, Preger et al. 2020 found that NMC fade is driven mainly by T, depth of discharge and C-rate.

**Online signal.** A relative rate with respect to a reference (25 °C, 50 % SOC, rest):

```
a = f_T(Tc) · f_SOC(SOC) + f_cyc(|I|, Tc)
f_T   = exp( (Ea/R)(1/298.15 − 1/Tc) ),  Ea ≈ 50 kJ/mol (assumption; fit to Kirkaldy 10/25/40 °C)
f_SOC = 1 (<60 %), 1.5 (60-90 %), 2.5 (>90 %)    (shape from Keil 2016; values to fit)
f_cyc = Ah-throughput term ∝ C-rate, plus 2× weight below 30 % SOC (Si loss, Kirkaldy 2022†)
```

Also accumulate: equivalent full cycles (EFC = ∫|I|dt / 2Q), Ah throughput, time above 90 % SOC and time above 40 °C.

**Role.** **Display only.** Lowering the current now does not undo calendar stress, and the factors are too uncertain to drive a safety limit. This gauge most directly answers the user's question "what adds stress and what doesn't". Parked at 100 % SOC with a 40 °C core it reads about 6-7× (f_T ≈ 2.6, f_SOC = 2.5). Gentle driving at 50 % SOC and 25 °C reads about 1×. Fast charging at 10 °C shows the cyclic term rising. The dashboard can show it as one number with its dominant factor, like Ψ.

**Thesis framing.** "Instantaneous ageing-rate multiplier, after Xu et al. 2018". The coefficients are explicitly flagged as indicative, with the M50T calendar/cycle data (Kirkaldy 2024; possibly Wildfeuer et al. 2023) as the route to fitting them.

### 3.4 Ω: monitoring integrity (display only, answers the bench observation)

**Mechanism.** This is not cell stress. Functional-safety practice separates *hazards from the battery* (over-T, over-V, under-V, over-I) from *loss of the means to detect them* (sensor, wire, communication, MCU faults). The second group triggers a safe state through diagnostics, not a stress estimate (Lelie et al. 2018; ISO 26262 safe-state concept). Our design already does this: LIMP and SAFE outside Ψ. That is correct.

**Online signal.** Flags the ECU already has: `T_valid`, `I_valid`, `CAN_ok`, CRC/counter failures, group-frame staleness, reset cause (IWDG), flash CRC.

**Role.** **Display only, and in the dashboard instead of in Ψ.**
- Each Ψ_i whose input is invalid is drawn **grey with "unknown"**, not 0. Ψ_T, Ψ_Tdot and Ψ_C are unknown when T is cut. Every criterion is unknown when the 0x101/0x102 line is cut. After a watchdog reset, all criteria are unknown until the EKF has converged.
- A separate "BMS confidence" strip names the lost signal and the resulting action ("temperature sensor open → LIMP, current capped at X A").

This removes the false impression "sensor cut = no stress". Showing **unknown** is the safe reading, and it is why the BMS limits the current.

### 3.5 Ψ_SD: abnormal self-discharge / internal-short precursor (later)

**Mechanism.** A developing soft internal short drains one parallel group, so its voltage drifts *relative to the pack mean* during rest. Consistency-based detection (cell deviation from the mean, the "mean-difference model") is the standard online approach for packs and is one of the few early warnings for internal-short-initiated thermal runaway (Ouyang et al. 2015†; Feng et al. 2016; Feng et al. 2018).

**Online signal.** At rest (|I| < 2 A, balancing off for that group), the group deviation `e_i = V_i − mean(V)`. Its slope over a window of tens of minutes, corrected for balancing (`BAL_*` flags known), is the drift. The static ΔV that was rightly dropped is not the signal; its **rate** is.

**Order of magnitude.** One 12P group = 58 Ah. A 10 Ω short draws about 0.37 A ≈ 0.64 % SOC/h. With dOCV/dSOC ≈ 5-10 mV/% in mid SOC, that is a drift of about 3-6 mV/h. This is detectable with 1 mV resolution over about 1 h of rest. 100 Ω gives about 0.3-0.6 mV/h, which is marginal.

**Thresholds (assumed).** Warning at 2 mV/h sustained for 1 h. LIMP at 10 mV/h.

**Role.** Warning and display first. A safety action comes only after validation, because the twin cannot produce the fault yet. It needs a new EMU fault (a group leak resistor, the same kind of mechanism as 0x210 balancing) and a long rest test. That makes it the most expensive item; rank 5.

---

## 4. Signals the ECU has → criteria map

| criterion | I | V group | T surf | T amb | SOC | k_R | Tc | new state |
|---|---|---|---|---|---|---|---|---|
| Ψ_V | ✓ | ✓ (V_min/V_max) | | | (check) | ✓ (r0) | ✓ (Arrhenius r0) | none |
| Ψ_P | ✓ | | | | ✓ | | ✓ | none |
| A | ✓ | | | | ✓ | | ✓ | EFC, Ah, dwell timers (a few floats) |
| Ω | flags | flags | flags | | | | | none (telemetry only) |
| Ψ_SD | ✓ (rest) | ✓ (19) | | | ✓ | | | 19 drift filters (~19×2 floats) |

All of them fit the `bms_step` budget (measured max 92 µs). None needs a CAN layout change. Ψ_i reach the dashboard through the DX UART line as the existing criteria do.

---

## 5. Required structural change: split Ψ by direction

```
common   = {T, C, Tdot, H}
Ψ_chg = agg(common ∪ {V_hi, P})       ;  I_allow_chg = min(I_ds_chg·(1 − Ψ_chg), SOP_chg)
Ψ_dis = agg(common ∪ {V_lo})          ;  I_allow_dis = min(I_ds_dis·(1 − Ψ_dis), SOP_dis)
agg(x) = min(1, max x + κ(Σx − max x)),  κ = 0.25 (unchanged)
```

With only the common criteria active, Ψ_chg = Ψ_dis = today's Ψ. Existing results (MC 1000/1000, PIL refs) should reproduce bit-for-bit until a directional criterion becomes non-zero. That is a useful regression check. The dashboard shows both bars, or one bar for the direction the current is flowing.

Mirror points, as for every `bms/common` change: `twin/decision.py`, `simulink/decision_logic.m` and `bms_param_struct.m`, SIL refs (`tools/pil_test/sil_ref.py`), DX telemetry fields, and the web panel.

---

## 6. Criteria considered and not recommended for Ψ

| criterion | why not (literature + our data) | where it goes instead |
|---|---|---|
| C-rate / I²t (Ψ_I) | Its harm acts through heat (already Ψ_T, Ψ_Tdot) and through plating (Ψ_P, charge only). Discharge C-rate up to 1.5C is within the M50T rating, and Preger 2020 shows the C-rate effect on NMC is secondary to T. Our MC probe: I²t ≤ 49 %, because the current is already clipped by I_allow (a circular criterion). | f_cyc term of gauge A |
| Static group ΔV (Ψ_B) | Nominal ΔV is 18-46 mV from production spread (Rumpf et al. 2017 measured this spread on 1100 cells). Balancing corrects it, and derating does not. Its real consequence, one group reaching a voltage limit first, is captured by Ψ_V because Ψ_V uses V_max/V_min of the groups. | display with balancing state; its *rate* → Ψ_SD |
| σ_SOC (Ψ_U) | MC: σ_SOC ≤ 0.75 %, the filter stays confident even when biased. An overconfident EKF makes it uninformative (Plett 2004 assumes a consistent filter). | dropped |
| Depth of discharge (rainflow) | Needs completed half-cycles, so it is an offline or cumulative quantity (Xu et al. 2018). Lowering current now does not change it. | cumulative histogram/EFC in gauge A |
| Ah throughput / EFC | Ageing *budget*, not instantaneous stress (Wang et al. 2011 Ah-throughput model). | gauge A counters, Ψ_H already reacts to its consequence |
| High-SOC dwell (calendar) | Real and large (Keil 2016), but current derating cannot relieve it. A BMS acts on it through charge-target policy, not current. | gauge A; possible later feature: charge-to-80 % recommendation |
| Sensor / line / watchdog faults | Not cell stress (Section 3.4). | Ω layer, LIMP/SAFE as now |

---

## 7. Suggested order of work (for the user to approve first)

1. **Ω display layer** (panel and telemetry only; no algorithm change). Fixes the "no stress" impression in today's button tests.
2. **Ψ split by direction** with no new criteria. Regression: MC and PIL must match today.
3. **Ψ_V**, then **Ψ_P**. Validate with A2-short (SOC 88 % fast charge) and a cold charge (`t 5`, mode 2, low SOC → high SOC). Add a low-SOC drive for Ψ_V discharge.
4. **Gauge A** in panel and telemetry. Coefficients flagged as indicative.
5. **Ψ_SD** after a new EMU leak-fault command exists (`bms_can.h` CAN_CMD → 3 Arduino copies).

All thresholds marked "assumption" should go through the same MC sensitivity check used for Ψ_Tdot and Ψ_H: no false activation in nominal runs.

---

## References

- Birkl, C.R., Roberts, M.R., McTurk, E., Bruce, P.G., Howey, D.A. (2017). Degradation diagnostics for lithium ion cells. *J. Power Sources* 341, 373-386. doi:10.1016/j.jpowsour.2016.12.011
- Edge, J.S. et al. (2021). Lithium ion battery degradation: what you need to know. *Phys. Chem. Chem. Phys.* 23, 8200-8221. doi:10.1039/D1CP00359C †
- Feng, X., Ouyang, M., Liu, X., Lu, L., Xia, Y., He, X. (2018). Thermal runaway mechanism of lithium ion battery for electric vehicles: A review. *Energy Storage Mater.* 10, 246-267. doi:10.1016/j.ensm.2017.05.013
- Feng, X., Weng, C., Ouyang, M., Sun, J. (2016). Online internal short circuit detection for a large format lithium ion battery. *Appl. Energy* 161, 168-180. doi:10.1016/j.apenergy.2015.10.019 †
- Guo, R., Lu, L., Ouyang, M., Feng, X. (2016). Mechanism of the entire overdischarge process and overdischarge-induced internal short circuit in lithium-ion batteries. *Sci. Rep.* 6, 30248. doi:10.1038/srep30248
- Jung, R., Metzger, M., Maglia, F., Stinner, C., Gasteiger, H.A. (2017). Oxygen release and its effect on the cycling stability of LiNixMnyCozO2 (NMC) cathode materials for Li-ion batteries. *J. Electrochem. Soc.* 164, A1361-A1377. doi:10.1149/2.0021707jes
- Keil, P., Schuster, S.F., Wilhelm, J., Travi, J., Hauser, A., Karl, R.C., Jossen, A. (2016). Calendar aging of lithium-ion batteries I. Impact of the graphite anode on capacity fade. *J. Electrochem. Soc.* 163, A1872-A1880. doi:10.1149/2.0411609jes
- Kirkaldy, N., Samieian, M.A., Offer, G.J., Marinescu, M., Patel, Y. (2022). Lithium-ion battery degradation: measuring rapid loss of active silicon in silicon-graphite composite electrodes. *ACS Appl. Energy Mater.* 5, 13367-13376. doi:10.1021/acsaem.2c02047 † (LG M50T)
- Kirkaldy, N. et al. (2024). Lithium-ion battery degradation: Comprehensive cycle ageing data and analysis for commercial 21700 cells. *J. Power Sources* 603, 234185. doi:10.1016/j.jpowsour.2024.234185 (journal/volume/article † ; DOI string to confirm) (LG M50T)
- Lelie, M., Braun, T., Knips, M., Nordmann, H., Ringbeck, F., Zappen, H., Sauer, D.U. (2018). Battery management system hardware concepts: an overview. *Appl. Sci.* 8, 534. doi:10.3390/app8040534
- Ouyang, M., Zhang, M., Feng, X., Lu, L., Li, J., He, X., Zheng, Y. (2015). Internal short circuit detection for battery pack using equivalent parameter and consistency method. *J. Power Sources* 294, 272-283. doi:10.1016/j.jpowsour.2015.06.087 †
- Plett, G.L. (2004). High-performance battery-pack power estimation using a dynamic cell model. *IEEE Trans. Veh. Technol.* 53, 1586-1593. doi:10.1109/TVT.2004.832408
- Preger, Y. et al. (2020). Degradation of commercial lithium-ion cells as a function of chemistry and cycling conditions. *J. Electrochem. Soc.* 167, 120532. doi:10.1149/1945-7111/abae37
- Rumpf, K., Naumann, M., Jossen, A. (2017). Experimental investigation of parametric cell-to-cell variation and correlation based on 1100 commercial lithium-ion cells. *J. Energy Storage* 14, 224-243. doi:10.1016/j.est.2017.09.010
- Ryu, H.-H., Park, K.-J., Yoon, C.S., Sun, Y.-K. (2018). Capacity fading of Ni-rich Li[NixCoyMn1−x−y]O2 (0.6 ≤ x ≤ 0.95) cathodes for high-energy-density lithium-ion batteries: bulk or surface degradation? *Chem. Mater.* 30, 1155-1163. doi:10.1021/acs.chemmater.7b05269
- Schmalstieg, J., Käbitz, S., Ecker, M., Sauer, D.U. (2014). A holistic aging model for Li(NiMnCo)O2 based 18650 lithium-ion batteries. *J. Power Sources* 257, 325-334. doi:10.1016/j.jpowsour.2014.02.012 (pre-2015, classic)
- Smith, K., Saxon, A., Keyser, M., Lundstrom, B., Cao, Z., Roc, A. (2017). Life prediction model for grid-connected Li-ion battery energy storage system. *Proc. American Control Conf.*, 4062-4068. doi:10.23919/ACC.2017.7963578
- Tomaszewska, A. et al. (2019). Lithium-ion battery fast charging: A review. *eTransportation* 1, 100011. doi:10.1016/j.etran.2019.100011
- Waldmann, T., Wilka, M., Kasper, M., Fleischhammer, M., Wohlfahrt-Mehrens, M. (2014). Temperature dependent ageing mechanisms in Lithium-ion batteries – A Post-Mortem study. *J. Power Sources* 262, 129-135. doi:10.1016/j.jpowsour.2014.03.112 (pre-2015, classic)
- Waldmann, T., Hogg, B.-I., Wohlfahrt-Mehrens, M. (2018). Li plating as unwanted side reaction in commercial Li-ion cells – A review. *J. Power Sources* 384, 107-124. doi:10.1016/j.jpowsour.2018.02.063
- Wang, J. et al. (2011). Cycle-life model for graphite-LiFePO4 cells. *J. Power Sources* 196, 3942-3948. doi:10.1016/j.jpowsour.2010.11.134 (Ah-throughput model, pre-2015)
- Wildfeuer, L., Karger, A., Aygül, D., Wassiliadis, N., Jossen, A., Lienkamp, M. (2023). Experimental degradation study of a commercial lithium-ion battery. *J. Power Sources* 560, 232498. doi:10.1016/j.jpowsour.2022.232498 † (check that the cell is the M50T before using its numbers)
- Xu, B., Oudalov, A., Ulbig, A., Andersson, G., Kirschen, D.S. (2018). Modeling of lithium-ion battery degradation for cell life assessment. *IEEE Trans. Smart Grid* 9, 1131-1140. doi:10.1109/TSG.2016.2578950
- Yang, X.-G., Zhang, G., Ge, S., Wang, C.-Y. (2018). Fast charging of lithium-ion batteries at all temperatures. *PNAS* 115, 7266-7271. doi:10.1073/pnas.1807115115
