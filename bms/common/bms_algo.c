/* bms_algo.c -- BMS kestirim ve karar algoritmaları (STM32 + PC SIL ortak kodu).
 * Python referanslarıyla birebir aynı denklemler ve aynı çağrı sırası (bms_step).
 */
#include <math.h>
#include <string.h>
#include "bms_config.h"
#include "bms_algo.h"

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float maxf(float a, float b) { return a > b ? a : b; }
static float minf(float a, float b) { return a < b ? a : b; }

static float polyval(const float *p, int n, float x)
{
    float y = p[0];
    for (int k = 1; k < n; k++) y = y * x + p[k];
    return y;
}
static float dpolyval(const float *p, int n, float x)      /* türev */
{
    float y = 0.0f;
    for (int k = 0; k < n - 1; k++) y = y * x + p[k] * (float)(n - 1 - k);
    return y;
}
static float arrhenius(float T)
{
    return expf(CELL_EA_R0 / RGAS * (1.0f / (T + 273.15f) - 1.0f / 298.15f));
}
static float interp_tab(const float *xs, const float *ys, int n, float x)
{   /* np.interp(x, xs, ys, left=0, right=0) */
    if (x < xs[0] || x > xs[n - 1]) return 0.0f;
    for (int k = 0; k < n - 1; k++)
        if (x <= xs[k + 1]) {
            float w = (x - xs[k]) / (xs[k + 1] - xs[k]);
            return ys[k] + w * (ys[k + 1] - ys[k]);
        }
    return ys[n - 1];
}

/* =================================================================== SOC EKF */
void ekf_init(ekf_t *e, float soc0, float q_scale, float r_scale)
{
    const float p0[4] = EKF_P0;
    const float k = (float)BMS_NS / (float)BMS_NP;
    memset(e, 0, sizeof(*e));
    e->x[0] = soc0;
    for (int i = 0; i < 4; i++) e->P[i][i] = p0[i];
    e->r0_ref = r_scale * CELL_R0 * k;
    e->r1 = r_scale * CELL_R1 * k;
    e->r2 = r_scale * CELL_R2 * k;
    e->a1 = expf(-BMS_TS / (e->r1 * (CELL_C1 / k)));
    e->a2 = expf(-BMS_TS / (e->r2 * (CELL_C2 / k)));
    e->q_ah = q_scale * CELL_Q_AH * BMS_NP;
}

void ekf_step(ekf_t *e, float i_meas, float v_meas, float T_use, float *innov, uint8_t *rejected)
{
    const float qd[4] = EKF_Q;
    float A[4][4] = {{0}}, Pp[4][4], AP[4][4], C[4], PC[4], K[4], xp[4];
    const float r0 = e->r0_ref * arrhenius(T_use);
    const float i = i_meas - e->x[3];
    const float kc = BMS_TS / (3600.0f * e->q_ah);

    /* ---- tahmin ---- */
    xp[0] = e->x[0] - kc * i;
    xp[1] = e->a1 * e->x[1] + e->r1 * (1.0f - e->a1) * i;
    xp[2] = e->a2 * e->x[2] + e->r2 * (1.0f - e->a2) * i;
    xp[3] = e->x[3];
    A[0][0] = 1.0f;  A[1][1] = e->a1;  A[2][2] = e->a2;  A[3][3] = 1.0f;
    A[0][3] = kc;  A[1][3] = -e->r1 * (1.0f - e->a1);  A[2][3] = -e->r2 * (1.0f - e->a2);
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += A[r][k] * e->P[k][c];
            AP[r][c] = s;
        }
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += AP[r][k] * A[c][k];
            Pp[r][c] = s + (r == c ? qd[r] : 0.0f);
        }

    /* ---- düzeltme ---- */
    const float s_cl = clampf(xp[0], 0.0f, 1.0f);
    const float y_hat = BMS_NS * polyval(OCV_POLY, OCV_POLY_N, s_cl) - r0 * i - xp[1] - xp[2];
    C[0] = BMS_NS * dpolyval(OCV_POLY, OCV_POLY_N, s_cl);  C[1] = -1.0f;  C[2] = -1.0f;  C[3] = r0;
    float S = EKF_R;
    for (int r = 0; r < 4; r++) {
        float s = 0.0f;
        for (int c = 0; c < 4; c++) s += Pp[r][c] * C[c];
        PC[r] = s;
        S += C[r] * s;
    }
    for (int r = 0; r < 4; r++) K[r] = PC[r] / S;
    *innov = v_meas - y_hat;

    *rejected = (fabsf(*innov) > EKF_GATE * sqrtf(S)) && (e->n_rej < EKF_N_REJ);
    if (*rejected) {
        e->n_rej++;
        memcpy(e->x, xp, sizeof(xp));
        memcpy(e->P, Pp, sizeof(Pp));
        return;
    }
    e->n_rej = 0;
    for (int r = 0; r < 4; r++) e->x[r] = xp[r] + K[r] * (*innov);
    /* Joseph formu: P = (I-KC) Pp (I-KC)' + R K K' */
    float IKC[4][4], T1[4][4];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) IKC[r][c] = (r == c ? 1.0f : 0.0f) - K[r] * C[c];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += IKC[r][k] * Pp[k][c];
            T1[r][c] = s;
        }
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += T1[r][k] * IKC[c][k];
            e->P[r][c] = s + EKF_R * K[r] * K[c];
        }
}

/* =================================================================== sıcaklık gözlemcisi */
void thermal_init(thermal_t *t) { memset(t, 0, sizeof(*t)); }

void thermal_start(thermal_t *t, float T_meas)
{   /* ilk ölçümle başlat: x = [T T 1/Ru], P = P0 (Python: ThermalEKF(t0=T)) */
    const float p0[3] = TH_P0;
    memset(t->P, 0, sizeof(t->P));
    t->x[0] = T_meas;  t->x[1] = T_meas;  t->x[2] = 1.0f / CELL_RU;
    for (int i = 0; i < 3; i++) t->P[i][i] = p0[i];
    t->started = 1;
}

void thermal_step(thermal_t *t, float i_meas, float v_meas, float soc_est, float T_meas,
                  float T_amb, uint8_t T_valid)
{
    const float dt = BMS_TS, cc = CELL_CC, cs = CELL_CS, rc = CELL_RC;
    if (!t->started) thermal_start(t, T_meas);
    const float qd[3] = TH_Q;
    const float tc = t->x[0], ts = t->x[1], g = t->x[2];
    const float ic = i_meas / BMS_NP;
    const float q = ic * (polyval(OCV_POLY, OCV_POLY_N, clampf(soc_est, 0.0f, 1.0f)) - v_meas / BMS_NS);
    float xp[3], A[3][3], AP[3][3], Pp[3][3];
    xp[0] = tc + dt / cc * (q + (ts - tc) / rc);
    xp[1] = ts + dt / cs * (g * (T_amb - ts) - (ts - tc) / rc);
    xp[2] = g;
    A[0][0] = 1.0f - dt / (cc * rc);  A[0][1] = dt / (cc * rc);  A[0][2] = 0.0f;
    A[1][0] = dt / (cs * rc);  A[1][1] = 1.0f - dt / cs * (g + 1.0f / rc);  A[1][2] = dt / cs * (T_amb - ts);
    A[2][0] = 0.0f;  A[2][1] = 0.0f;  A[2][2] = 1.0f;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += A[r][k] * t->P[k][c];
            AP[r][c] = s;
        }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += AP[r][k] * A[c][k];
            Pp[r][c] = s + (r == c ? qd[r] : 0.0f);
        }
    if (!T_valid) {                          /* bozulmuş mod: sadece model */
        memcpy(t->x, xp, sizeof(xp));
        memcpy(t->P, Pp, sizeof(Pp));
        return;
    }
    const float S = Pp[1][1] + TH_R;
    float K[3];
    for (int r = 0; r < 3; r++) K[r] = Pp[r][1] / S;
    const float innov = T_meas - xp[1];
    for (int r = 0; r < 3; r++) t->x[r] = xp[r] + K[r] * innov;
    t->x[2] = maxf(t->x[2], TH_G_MIN);
    float IKC[3][3], T1[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) IKC[r][c] = (r == c ? 1.0f : 0.0f) - (c == 1 ? K[r] : 0.0f);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += IKC[r][k] * Pp[k][c];
            T1[r][c] = s;
        }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += T1[r][k] * IKC[c][k];
            t->P[r][c] = s + TH_R * K[r] * K[c];
        }
}

/* =================================================================== sensör teşhisi */
void diag_init(diag_t *d)
{
    memset(d, 0, sizeof(*d));
    d->first = 1;  d->t_valid = 1;  d->i_valid = 1;
}

void diag_step(diag_t *d, float i_meas, float T_meas)
{
    if (!d->first) {
        d->n_t = (T_meas == d->t_prev) ? d->n_t + 1 : 0;   /* CAN değerleri: tam eşitlik anlamlı */
        d->n_i = (i_meas == d->i_prev) ? d->n_i + 1 : 0;
        if (fabsf(T_meas - d->t_prev) > DIAG_T_RATE) d->t_valid = 0;
    }
    if (T_meas < DIAG_T_MIN || T_meas > DIAG_T_MAX || d->n_t >= DIAG_N_FROZEN) d->t_valid = 0;
    if (d->n_i >= DIAG_N_FROZEN) d->i_valid = 0;
    d->first = 0;
    d->t_prev = T_meas;
    d->i_prev = i_meas;
}

/* =================================================================== direnç kestiricisi */
void rint_init(rint_t *r, float num0, float den0)
{
    memset(r, 0, sizeof(*r));
    r->num = num0;  r->den = den0;  r->first = 1;
}

float rint_step(rint_t *r, float i_meas, float v_meas, float Tc_hat, float soc_est, uint8_t I_valid)
{
    if (!r->first && I_valid) {
        const float di = i_meas - r->i_prev, dv = v_meas - r->v_prev;
        if (fabsf(di) >= RINT_DI_MIN && soc_est > RINT_SOC_MIN) {
            const float f = arrhenius(Tc_hat) * (1.0f + 0.6f * expf(-soc_est / 0.08f));
            const float k = (float)BMS_NP / (float)BMS_NS;
            const float r_ev = -dv / di * k / f;
            if (r_ev > 0.3f * CELL_R0 && r_ev < 4.0f * CELL_R0) {
                r->num = RINT_LAM * r->num + (-dv * di / f) * k;
                r->den = RINT_LAM * r->den + di * di;
                r->n++;
            }
        }
    }
    r->first = 0;
    r->i_prev = i_meas;
    r->v_prev = v_meas;
    return r->den > 0.0f ? r->num / r->den / CELL_R0 : 1.0f;
}

/* =================================================================== karar mekanizması */
void decision_init(decision_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = ST_NORMAL;                    /* I_allow 0'dan rampa ile başlar */
}

static uint8_t band(uint8_t f, float ratio)
{
    if (ratio < DEC_BAND_ON) return 1;
    if (ratio > DEC_BAND_OFF) return 0;
    return f;
}

void decision_step(decision_t *d, float I_meas, float V_min, float V_max, float Tc_hat,
                   float Ts_hat, float Ru_hat, float SOC_est, float bias_est,
                   uint8_t T_valid, uint8_t I_valid, uint8_t CAN_ok, float T_meas, float T_amb,
                   decision_out_t *o)
{
    const float I_nom_dis = 7.275f * BMS_NP, I_nom_chg = 3.395f * BMS_NP;

    /* ---- sert sınırlar (kilitlenir) ---- */
    d->n_lo = (V_min < DEC_V_SAFE_LO) ? d->n_lo + 1 : 0;
    d->n_hi = (V_max > DEC_V_SAFE_HI) ? d->n_hi + 1 : 0;
    if (T_valid) {                           /* ölçüme bağlı çekirdek sınırı */
        const float t_ss = T_meas + DEC_K_SS * (T_meas - T_amb);
        d->n_hl = (t_ss > DEC_T_LIMP) ? d->n_hl + 1 : 0;
        d->n_hs = (t_ss > DEC_T_SAFE) ? d->n_hs + 1 : 0;
    } else {
        d->n_hl = d->n_hs = 0;
    }
    if (Tc_hat > DEC_T_SAFE || d->n_hs >= DEC_N_HARD || d->n_lo >= DEC_N_HARD || d->n_hi >= DEC_N_HARD || !CAN_ok
        || d->state == ST_SAFE) {
        d->state = ST_SAFE;  d->i_dis = 0.0f;  d->i_chg = 0.0f;
        o->state = ST_SAFE;  o->i_allow_dis = 0.0f;  o->i_allow_chg = 0.0f;  o->psi = 1.0f;  o->cause = 128;
        return;
    }

    /* ---- veri sayfası sınırları ---- */
    const float i_ds_dis = interp_tab(DEC_T_DIS, DEC_I_DIS, DEC_DS_N, Ts_hat) * BMS_NP;
    const float i_ds_chg = interp_tab(DEC_T_CHG, DEC_I_CHG, DEC_DS_N, Ts_hat) * BMS_NP;
    d->f_ds = band(d->f_ds, minf(i_ds_dis / I_nom_dis, i_ds_chg / I_nom_chg));

    /* ---- stres indeksi ---- */
    if (Tc_hat >= DEC_T_DER_ON) d->therm_on = 1;
    else if (Tc_hat < DEC_T_DER_OFF) d->therm_on = 0;
    const float psi_T = d->therm_on ? clampf((Tc_hat - DEC_T_DER_OFF) / (DEC_T_DER_FULL - DEC_T_DER_OFF), 0.f, 1.f) : 0.0f;
    if (d->n_run < DEC_N_COOL_BLANK) d->n_run++;
    if (d->n_run < DEC_N_COOL_BLANK) d->cool_on = 0;          /* gözlemci yakınsıyor */
    else if (Ru_hat > DEC_RU_ALARM) d->cool_on = 1;
    else if (Ru_hat < DEC_RU_ALARM_OFF) d->cool_on = 0;
    const float psi_C = d->cool_on ? DEC_PSI_COOL : 0.0f;
    const float psi = maxf(psi_T, psi_C);

    /* ---- gerilim payı (SOP) ---- */
    const float r0 = CELL_R0 * arrhenius(Tc_hat);
    const float sop_dis = maxf(I_meas, 0.f) + DEC_K_SOP * BMS_NP * (V_min - DEC_V_LIM_DIS) / r0;
    const float sop_chg = maxf(-I_meas, 0.f) + DEC_K_SOP * BMS_NP * (DEC_V_LIM_CHG - V_max) / r0;
    const float r_dc = r0 + DEC_R_RC, ic = I_meas / BMS_NP;
    const float qs_dis = BMS_NP * (V_min + maxf(ic, 0.f) * r0 - DEC_V_LIM_DIS) / r_dc;
    const float qs_chg = BMS_NP * (DEC_V_LIM_CHG - (V_max - maxf(-ic, 0.f) * r0)) / r_dc;
    d->f_sop_dis = band(d->f_sop_dis, qs_dis / maxf(i_ds_dis, 1e-6f));
    d->f_sop_chg = band(d->f_sop_chg, qs_chg / maxf(i_ds_chg, 1e-6f));

    float tgt_dis = maxf(0.f, minf(i_ds_dis * (1.f - psi), sop_dis));
    float tgt_chg = maxf(0.f, minf(i_ds_chg * (1.f - psi), sop_chg));

    /* ---- LIMP-HOME ---- */
    const uint8_t sensor_bad = !T_valid || !I_valid || fabsf(bias_est) > DEC_BIAS_MAX;
    const uint8_t empty = SOC_est < DEC_SOC_EMPTY;
    const uint8_t hot_ss = d->n_hl >= DEC_N_HARD;
    const uint8_t limp = sensor_bad || Tc_hat > DEC_T_LIMP || hot_ss || empty;
    if (limp) {
        tgt_dis = minf(minf(DEC_FLOOR * I_nom_dis, i_ds_dis), maxf(sop_dis, 0.f));
        tgt_chg = minf(tgt_chg, DEC_FLOOR * I_nom_chg);
    }
    const uint16_t cause = (psi_T > 0.f || hot_ss ? 1 : 0) | (psi_C > 0.f ? 2 : 0) | (d->f_sop_dis ? 4 : 0)
                         | (d->f_sop_chg ? 8 : 0) | (d->f_ds ? 16 : 0) | (sensor_bad ? 32 : 0)
                         | (empty ? 64 : 0);

    /* ---- durum + histerezis ---- */
    const uint8_t want = limp ? ST_LIMP : (cause ? ST_DERATE : ST_NORMAL);
    if (want > d->state) {
        d->state = want;  d->n_ok = 0;
    } else if (want < d->state) {
        if (++d->n_ok >= DEC_N_EXIT) { d->state = want;  d->n_ok = 0; }
    } else {
        d->n_ok = 0;
    }

    /* ---- rampa: düşüş anında, artış <= DEC_RAMP_UP ---- */
    const float step = DEC_RAMP_UP * BMS_TS;
    d->i_dis = (tgt_dis < d->i_dis) ? tgt_dis : minf(tgt_dis, d->i_dis + step);
    d->i_chg = (tgt_chg < d->i_chg) ? tgt_chg : minf(tgt_chg, d->i_chg + step);

    o->state = d->state;  o->i_allow_dis = d->i_dis;  o->i_allow_chg = d->i_chg;
    o->psi = 1.f - d->i_dis / I_nom_dis;  o->cause = cause;
}

/* =================================================================== uygulama */
void bms_init(bms_t *b, float soc0_ekf, float q_scale, float r_scale, float rint_num0, float rint_den0)
{
    ekf_init(&b->ekf, soc0_ekf, q_scale, r_scale);
    thermal_init(&b->th);
    diag_init(&b->diag);
    rint_init(&b->rint, rint_num0, rint_den0);
    decision_init(&b->dec);
}

/* Çağrı sırası Python referansıyla aynı (twin/closed_loop.py):
 * teşhis -> SOC EKF (T geçersizse gözlemcinin önceki T_surf'ü) -> sıcaklık gözlemcisi
 * -> direnç kestiricisi -> karar mekanizması */
void bms_step(bms_t *b, const bms_in_t *in, bms_out_t *o)
{
    diag_step(&b->diag, in->I, in->T);
    o->t_valid = b->diag.t_valid;
    o->i_valid = b->diag.i_valid;

    if (!b->th.started) thermal_start(&b->th, in->T);
    const float T_use = o->t_valid ? in->T : b->th.x[1];
    ekf_step(&b->ekf, in->I, in->V, T_use, &o->innov, &o->rejected);
    o->soc = b->ekf.x[0];
    o->bias = b->ekf.x[3];

    thermal_step(&b->th, in->I, in->V, o->soc, in->T, in->T_amb, o->t_valid);
    o->tc = b->th.x[0];  o->ts = b->th.x[1];  o->ru = 1.0f / b->th.x[2];

    o->rr = rint_step(&b->rint, in->I, in->V, o->tc, o->soc, o->i_valid);

    decision_step(&b->dec, in->I, in->V_min, in->V_max, o->tc, o->ts, o->ru, o->soc, o->bias,
                  o->t_valid, o->i_valid, in->can_ok, in->T, in->T_amb, &o->dec);
}
