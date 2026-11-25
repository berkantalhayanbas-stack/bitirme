/* can_sil.c -- ham CAN kaydının PC (SIL) sürümü: kart (APP_ECU, BMS_CANLOG=1) olmadan aynı "C," satırları.
 * İkiz (bms/sim/twin_model.c) + ECU zinciri (bms_step, soh_tree, bal_step) + bms/common/bms_can.c
 * kodlayıcıları: kartla aynı kod, aynı çerçeve düzeni, sayaç ve CRC. Hat zamanlaması ideal (10 Hz, 1 ms
 * çözünürlük, sabit sıra); gerçek tezgâh kaydı değildir, "SIL" diye etiketlenmeli.
 *
 * Kullanım: can_sil <süre_s> <senaryo> [soc0] [t_amb]   -> stdout: C,<t_ms>,<R|T>,<id>,<veri>
 *   drive : sürüş çevrimi (mc_cycle.h), ECU izniyle kırpılır
 *   park  : 0 A, pasif dengeleme (twin_emu mod 3, b 500 ile hızlandırılmış)
 *   ntc   : sürüş, 60 s'de NTC kopuk (0x101 T = -40) -> LIMP
 *   heat  : sürüş, 60 s'de dış ısı 0,5 °C/s 62 °C'ye -> termal anomali LIMP -> SAFE
 * tools/can/can_log.py --sil <senaryo> derler, koşturur, candump yazar. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "twin_model.h"
#include "mc_cycle.h"
#include "bms_algo.h"
#include "bms_bal.h"
#include "bms_can.h"
#include "bms_config.h"
#include "soh_tree.h"

/* gürültü: mc_core.c ile aynı xorshift32 ve ~N(0,1) */
static uint32_t rng = 12345u;
static float urand(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return (float)(rng >> 8) * (1.0f / 16777216.0f); }
static float grand(void) { return (urand() + urand() + urand() + urand() - 2.0f) * 1.7320508f; }

static float cycle_i(float t_s)
{
    const uint32_t s = (uint32_t)t_s;
    const float f = t_s - (float)s;
    const float a = MC_CYCLE[s % MC_CYCLE_N], b = MC_CYCLE[(s + 1u) % MC_CYCLE_N];
    return a + (b - a) * f;
}

static void line(uint32_t t_ms, char dir, uint16_t id, const uint8_t d[8])
{
    printf("C,%u,%c,%03X,", (unsigned)t_ms, dir, id);
    for (int i = 0; i < 8; i++) printf("%02X", d[i]);
    putchar('\n');
}

int main(int argc, char **argv)
{
    const float dur = argc > 1 ? (float)atof(argv[1]) : 120.0f;
    const char *sc = argc > 2 ? argv[2] : "drive";
    const float soc0 = argc > 3 ? (float)atof(argv[3]) : (strcmp(sc, "park") == 0 ? 0.60f : 0.80f);
    const float t_amb = argc > 4 ? (float)atof(argv[4]) : 25.0f;
    const int park = strcmp(sc, "park") == 0, ntc = strcmp(sc, "ntc") == 0, heat = strcmp(sc, "heat") == 0;
    const float k_soc = park ? 500.0f : 1.0f;      /* dengelemenin SOC etkisi (twin_emu "b 500") */
    const float t_fault = 60.0f;

    static twin_t tw;
    static bms_t bms;
    static bal_t bal;
    twin_init(&tw, soc0, t_amb, BMS_TS);
    bms_init(&bms, 0.5f, 1.0f, 1.0f, 0.0f, 0.0f);  /* app_ecu ile aynı: EKF %50'den */
    bal_init(&bal);

    uint8_t c101 = 0, c102 = 0, cc[CAN_N_CELL_FRAMES] = { 0 }, c201 = 0, c202 = 0, c210 = 0;
    float i_dis = 0.0f, i_chg = 0.0f;
    uint32_t mask = 0;
    bms_out_t o;
    memset(&o, 0, sizeof o);
    const uint32_t n = (uint32_t)(dur / BMS_TS);

    for (uint32_t k = 0; k < n; k++) {
        const float t = (float)k * BMS_TS;
        const uint32_t t0 = 100u + 100u * k;       /* ECU açılışındaki 50 ms'lik atma süresinden sonra */
        /* ---- ikiz (EMU): akım talebi ECU izniyle kırpılır */
        float I = park ? 0.0f : cycle_i(t);
        if (I > i_dis) I = i_dis;
        if (I < -i_chg) I = -i_chg;
        twin_step(&tw, I, mask, tw.v_max / 10.0f, k_soc);       /* R_BLEED 10 ohm (twin_emu) */
        if (heat && t >= t_fault && tw.ts < 62.0f) {
            const float e = 62.0f - tw.ts, d = e > 0.05f ? 0.05f : e;
            tw.ts += d;  tw.tc += d;
        }
        /* ---- EMU -> ECU: 0x101, 0x102, 0x110-0x114 (closed_loop.py / twin_emu gürültüsü) */
        const can_meas1_t m1 = { tw.v_pack + 0.25f * grand(), I + 0.5f * grand(),
                                 (ntc && t >= t_fault) ? -40.0f : tw.ts + 0.3f * grand() };
        const can_meas2_t m2 = { tw.v_min + 0.005f * grand(), tw.v_max + 0.005f * grand(), t_amb };
        uint8_t d[8], d1[8], d2[8];
        can_encode_meas1(&m1, c101, d1);  c101 = (c101 + 1u) & 0x0Fu;
        can_encode_meas2(&m2, c102, d2);  c102 = (c102 + 1u) & 0x0Fu;
        line(t0, 'R', CAN_ID_EMU_MEAS1, d1);
        line(t0 + 1u, 'R', CAN_ID_EMU_MEAS2, d2);
        float vcell[BMS_NS];
        for (uint32_t f = 0; f < CAN_N_CELL_FRAMES; f++) {
            can_cell_t g;
            memset(&g, 0, sizeof g);
            for (uint32_t j = 0; j < CAN_CELLS_PER_FRAME; j++) {
                const uint32_t gi = 4u * f + j;
                if (gi >= TW_NG) break;
                g.v[j] = tw.g[gi].v + 0.001f * grand();
                g.valid |= (uint8_t)(1u << j);
                if (mask & (1u << gi)) g.bal |= (uint8_t)(1u << j);
            }
            can_encode_cell((uint8_t)f, &g, cc[f], d);  cc[f] = (cc[f] + 1u) & 0x0Fu;
            line(t0 + 3u + f, 'R', (uint16_t)(CAN_ID_EMU_CELL0 + f), d);
            can_cell_t back;  uint8_t cnt;                       /* ECU'nun gördüğü (kuantalanmış) değer */
            can_decode_cell((uint8_t)f, d, &back, &cnt);
            for (uint32_t j = 0; j < CAN_CELLS_PER_FRAME && 4u * f + j < BMS_NS; j++) vcell[4u * f + j] = back.v[j];
        }
        /* ---- ECU: çözülen değerlerle adım (kartta olduğu gibi CAN kuantalaması dahil) */
        can_meas1_t a;  can_meas2_t b;  uint8_t cnt;
        can_decode_meas1(d1, &a, &cnt);
        can_decode_meas2(d2, &b, &cnt);
        const bms_in_t in = { a.I, a.V, a.T, b.V_min, b.V_max, b.T_amb, 1u };
        bms_step(&bms, &in, &o);
        const float soh = soh_tree(0.0f, 0.0f, o.tc, o.soc, 0.0f, o.rr);
        i_dis = o.dec.i_allow_dis;  i_chg = o.dec.i_allow_chg;
        const can_ctrl_t ctrl = { o.dec.i_allow_dis, o.dec.i_allow_chg, o.dec.state, (uint8_t)o.dec.cause };
        can_encode_ctrl(&ctrl, c201, d);  c201 = (c201 + 1u) & 0x0Fu;
        line(t0 + 2u, 'T', CAN_ID_ECU_CTRL, d);
        const can_status_t st = { 100.0f * o.soc, o.tc, 100.0f * soh, o.dec.psi };
        can_encode_status(&st, c202, d);  c202 = (c202 + 1u) & 0x0Fu;
        line(t0 + 2u, 'T', CAN_ID_ECU_STATUS, d);
        mask = bal_step(&bal, vcell, 1u, a.I, o.dec.state, o.tc);
        const can_bal_t bm = { mask, bal.flags, bal.v_max - bal.v_min };
        can_encode_bal(&bm, c210, d);  c210 = (c210 + 1u) & 0x0Fu;
        line(t0 + 2u, 'T', CAN_ID_ECU_BAL, d);
    }
    return 0;
}
