/* bms_bal.c -- pasif hücre dengeleme kararı (açıklama bms_bal.h'de). */
#include <string.h>
#include "bms_bal.h"
#include "bms_algo.h"
#include "bms_can.h"

#define BAL_N_ON  ((uint16_t)(BAL_T_ON / BMS_TS + 0.5f))

void bal_init(bal_t *b)
{
    memset(b, 0, sizeof(*b));
}

uint32_t bal_step(bal_t *b, const float v[BMS_NS], uint8_t v_ok, float I, uint8_t state, float tc_hat)
{
    if (!v_ok) {
        b->have_vf = 0;  b->n_ok = 0;  b->mask = 0;
        b->flags = CAN_BAL_F_STALE;
        return 0;
    }

    /* ---- süzgeç, en düşük / en yüksek ---- */
    float vmin = 10.0f, vmax = 0.0f;
    for (int i = 0; i < BMS_NS; i++) {
        b->vf[i] = b->have_vf ? b->vf[i] + BAL_FILT * (v[i] - b->vf[i]) : v[i];
        if (b->vf[i] < vmin) vmin = b->vf[i];
        if (b->vf[i] > vmax) vmax = b->vf[i];
    }
    b->have_vf = 1;
    b->v_min = vmin;  b->v_max = vmax;

    /* ---- izin ve açma beklemesi ---- */
    const uint8_t drive   = I > BAL_I_DRIVE;
    const uint8_t blocked = !(state == ST_NORMAL || state == ST_DERATE) || tc_hat > BAL_T_MAX || vmin < BAL_V_MIN;
    if (drive || blocked) b->n_ok = 0;
    else if (b->n_ok < BAL_N_ON) b->n_ok++;
    const uint8_t active = b->n_ok >= BAL_N_ON;

    /* ---- grup başına histerezis ---- */
    if (!active) {
        b->mask = 0;
    } else {
        for (int i = 0; i < BMS_NS; i++) {
            const float d = b->vf[i] - vmin;
            const uint32_t bit = 1ul << i;
            if (b->mask & bit) { if (d < BAL_DV_OFF) b->mask &= ~bit; }
            else if (d > BAL_DV_ON) b->mask |= bit;
        }
    }
    b->flags = (uint8_t)((active ? CAN_BAL_F_ACTIVE : 0) | (drive ? CAN_BAL_F_DRIVE : 0)
                         | (blocked ? CAN_BAL_F_BLOCKED : 0));
    return b->mask;
}
