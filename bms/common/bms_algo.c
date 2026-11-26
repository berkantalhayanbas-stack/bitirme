/* bms_algo.c -- BMS kestirim ve karar algoritmaları (STM32 + PC SIL ortak kodu).
 * Python referanslarıyla birebir aynı denklemler ve aynı çağrı sırası (bms_step).
 */
#include <math.h>
#include <string.h>
#include "bms_config.h"
#include "bms_algo.h"
#include "stress_model.h"

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
static float arrhenius_ea(float ea, float T)
{   /* R(T) / R(25 °C) */
    return expf(ea / RGAS * (1.0f / (T + 273.15f) - 1.0f / 298.15f));
}
static float arrhenius(float T) { return arrhenius_ea(CELL_EA_R0, T); }
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
    const float p0[EKF_N] = EKF_P0;
    const float k = (float)BMS_NS / (float)BMS_NP;
    memset(e, 0, sizeof(*e));
    e->x[0] = soc0;
    e->x[4] = 1.0f;
    for (int i = 0; i < EKF_N; i++) e->P[i][i] = p0[i];
    e->r0_ref = r_scale * CELL_R0 * k;
    e->r1 = r_scale * CELL_R1 * k;
    e->r2 = r_scale * CELL_R2 * k;
    e->q_ah = q_scale * CELL_Q_AH * BMS_NP;
}

void ekf_step(ekf_t *e, float i_meas, float v_meas, float T_use, float *innov, uint8_t *rejected)
{
    const float qd[EKF_N] = EKF_Q;
    float A[EKF_N][EKF_N] = {{0}}, Pp[EKF_N][EKF_N], AP[EKF_N][EKF_N], C[EKF_N], PC[EKF_N], K[EKF_N], xp[EKF_N];
    const float r0 = e->r0_ref * arrhenius(T_use);
    const float kp = (float)BMS_NS / (float)BMS_NP;
    const float r1 = e->r1 * arrhenius_ea(CELL_EA_R1, T_use);    /* RC dalları da sıcaklığa bağlı */
    const float r2 = e->r2 * arrhenius_ea(CELL_EA_R2, T_use);
    e->a1 = expf(-BMS_TS / (r1 * (CELL_C1 / kp)));
    e->a2 = expf(-BMS_TS / (r2 * (CELL_C2 / kp)));
    const float kr = e->x[4];
    const float i = i_meas - e->x[3];
    const float kc = BMS_TS / (3600.0f * e->q_ah);

    /* ---- tahmin ---- */
    xp[0] = e->x[0] - kc * i;
    xp[1] = e->a1 * e->x[1] + r1 * (1.0f - e->a1) * i;
    xp[2] = e->a2 * e->x[2] + r2 * (1.0f - e->a2) * i;
    xp[3] = e->x[3];
    xp[4] = kr;
    A[0][0] = 1.0f;  A[1][1] = e->a1;  A[2][2] = e->a2;  A[3][3] = 1.0f;  A[4][4] = 1.0f;
    A[0][3] = kc;  A[1][3] = -r1 * (1.0f - e->a1);  A[2][3] = -r2 * (1.0f - e->a2);
    for (int r = 0; r < EKF_N; r++)
        for (int c = 0; c < EKF_N; c++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) s += A[r][k] * e->P[k][c];
            AP[r][c] = s;
        }
    for (int r = 0; r < EKF_N; r++)
        for (int c = 0; c < EKF_N; c++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) s += AP[r][k] * A[c][k];
            Pp[r][c] = s + (r == c ? qd[r] : 0.0f);
        }

    /* ---- düzeltme ---- */
    const float s_cl = clampf(xp[0], 0.0f, 1.0f);
    const float y_hat = BMS_NS * polyval(OCV_POLY, OCV_POLY_N, s_cl) - kr * r0 * i - xp[1] - xp[2];
    C[0] = BMS_NS * dpolyval(OCV_POLY, OCV_POLY_N, s_cl);  C[1] = -1.0f;  C[2] = -1.0f;  C[3] = kr * r0;  C[4] = -r0 * i;
    float S = EKF_R;
    for (int r = 0; r < EKF_N; r++) {
        float s = 0.0f;
        for (int c = 0; c < EKF_N; c++) s += Pp[r][c] * C[c];
        PC[r] = s;
        S += C[r] * s;
    }
    for (int r = 0; r < EKF_N; r++) K[r] = PC[r] / S;
    if (e->n_kr < EKF_N_KR_FREEZE) { e->n_kr++;  K[4] = 0.0f; }   /* SOC yakınsıyor: k_R dondur (Joseph formu tutarlı) */
    *innov = v_meas - y_hat;

    *rejected = (fabsf(*innov) > EKF_GATE * sqrtf(S)) && (e->n_rej < EKF_N_REJ);
    if (*rejected) {
        e->n_rej++;
        memcpy(e->x, xp, sizeof(xp));
        memcpy(e->P, Pp, sizeof(Pp));
        e->x[0] = clampf(e->x[0], 0.0f, 1.0f);   /* SOC fiziksel aralıkta (aşağıdaki not) */
        return;
    }
    e->n_rej = 0;
    for (int r = 0; r < EKF_N; r++) e->x[r] = xp[r] + K[r] * (*innov);
    /* SOC [0, 1]: boş pakette yeniden başlatmada (EKF %50'den başlar) gerilim OCV'nin dibinde kalır, filtre
     * SOC'yi sıfırın altına itiyordu (kart, 2026-11-22: -%20,7; panelde ve 0x202'de görünür). Kırpma
     * kovaryansa dokunmaz (durum kısıtı, Simon 2010 "clipping"); "boş" kararı SOC < %3'te zaten verilir */
    e->x[0] = clampf(e->x[0], 0.0f, 1.0f);
    e->x[4] = clampf(e->x[4], EKF_KR_MIN, EKF_KR_MAX);
    /* Joseph formu: P = (I-KC) Pp (I-KC)' + R K K' */
    float IKC[EKF_N][EKF_N], T1[EKF_N][EKF_N];
    for (int r = 0; r < EKF_N; r++)
        for (int c = 0; c < EKF_N; c++) IKC[r][c] = (r == c ? 1.0f : 0.0f) - K[r] * C[c];
    for (int r = 0; r < EKF_N; r++)
        for (int c = 0; c < EKF_N; c++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) s += IKC[r][k] * Pp[k][c];
            T1[r][c] = s;
        }
    for (int r = 0; r < EKF_N; r++)
        for (int c = 0; c < EKF_N; c++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) s += T1[r][k] * IKC[c][k];
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

/* =================================================================== termal anomali */
void tanom_init(tanom_t *a) { memset(a, 0, sizeof(*a)); }

uint8_t tanom_step(tanom_t *a, float i_meas, float v_meas, float soc_est, float T_meas, float T_amb,
                   float Tc_hat, uint8_t T_valid)
{
    if (!T_valid) { a->started = 0;  a->n = 0;  return a->flag; }   /* ölçüm dönünce baştan */
    if (!a->started) {                       /* ilk adımda iki zarf da başlar */
        a->t_f = T_meas;  a->k = 0;  a->started = 1;  a->age[0] = a->age[1] = -1;
    }
    a->t_f += TA_ALPHA * (T_meas - a->t_f);
    const float ic = i_meas / BMS_NP;
    const float q = maxf(ic * (polyval(OCV_POLY, OCV_POLY_N, clampf(soc_est, 0.0f, 1.0f)) - v_meas / BMS_NS), 0.0f)
                  * TA_Q_K + TA_Q_OFF;
    float r = -1e9f;
    for (int j = 0; j < 2; j++) {
        if (a->k == j * (TA_WIN / 2) || a->age[j] < 0) {   /* kaydırmalı yeniden başlatma */
            a->tc[j] = maxf(Tc_hat, a->t_f) + TA_TC0;  a->ts[j] = a->t_f + TA_TS0;  a->age[j] = 0;
        }
        const float tc = a->tc[j], ts = a->ts[j], g = (T_amb > ts) ? TA_G_MAX : 0.0f;
        a->tc[j] = tc + BMS_TS / CELL_CC * (q + (ts - tc) / CELL_RC);
        a->ts[j] = ts + BMS_TS / CELL_CS * (g * (T_amb - ts) - (ts - tc) / CELL_RC);
        a->age[j]++;
        if (a->age[j] >= TA_AGE_MIN) r = maxf(r, a->t_f - a->ts[j]);
    }
    a->k = (a->k + 1) % TA_WIN;
    if (r > -1e8f) {
        a->n = (r > TA_R_TH) ? a->n + 1 : 0;
        if (a->n >= TA_N) a->flag = 1;
    }
    return a->flag;
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
        d->n_rate = (fabsf(T_meas - d->t_lp) > DIAG_T_RATE) ? d->n_rate + 1 : 0;
        if (d->n_rate >= DIAG_N_RATE) d->t_valid = 0;
        d->t_lp += DIAG_T_ALPHA * (T_meas - d->t_lp);
    } else {
        d->t_lp = T_meas;
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
void ekf_zero(ekf_t *e, float i_meas)
{
    if (e->n_zero >= DEC_N_ZERO) return;
    e->i0_sum += i_meas;
    if (++e->n_zero == DEC_N_ZERO) {
        e->x[3] = e->i0_sum / (float)DEC_N_ZERO;
        for (int j = 0; j < EKF_N; j++) { e->P[3][j] = 0.0f;  e->P[j][3] = 0.0f; }
        e->P[3][3] = EKF_P_B0;
    }
}

void decision_init(decision_t *d)
{
    memset(d, 0, sizeof(*d));
    d->state = ST_NORMAL;                    /* I_allow 0'dan rampa ile başlar */
    d->i0_ok = 1;
}

static float ocv_clamped(float soc)
{   /* np.interp(soc, SOH_OCV_SOC, SOH_OCV_V): uçlarda sabit */
    if (soc <= SOH_OCV_SOC[0]) return SOH_OCV_V[0];
    for (int k = 0; k < SOH_OCV_N - 1; k++)
        if (soc <= SOH_OCV_SOC[k + 1])
            return SOH_OCV_V[k] + (soc - SOH_OCV_SOC[k]) / (SOH_OCV_SOC[k + 1] - SOH_OCV_SOC[k])
                                  * (SOH_OCV_V[k + 1] - SOH_OCV_V[k]);
    return SOH_OCV_V[SOH_OCV_N - 1];
}

static float or2(float a, float b) { return 1.0f - (1.0f - a) * (1.0f - b); }

static uint8_t band(uint8_t f, float ratio)
{
    if (ratio < DEC_BAND_ON) return 1;
    if (ratio > DEC_BAND_OFF) return 0;
    return f;
}

void decision_step(decision_t *d, float I_meas, float V_min, float V_max, float Tc_hat,
                   float Ts_hat, float Ru_hat, float SOC_est, float bias_est,
                   uint8_t T_valid, uint8_t I_valid, uint8_t CAN_ok, float T_meas, float T_amb,
                   uint8_t T_anom, float k_R, float soh_c, decision_out_t *o)
{
    const float I_nom_dis = 7.275f * BMS_NP, I_nom_chg = 3.395f * BMS_NP;

    /* ---- çok kriterli stres indeksi (SAFE'te de hesaplanır: telemetri Ψ'yi her durumda gösterir) ---- */
    if (Tc_hat >= DEC_T_DER_ON) d->therm_on = 1;
    else if (Tc_hat < DEC_T_DER_OFF) d->therm_on = 0;
    const float psi_T = d->therm_on ? clampf((Tc_hat - DEC_T_DER_OFF) / (DEC_T_DER_FULL - DEC_T_DER_OFF), 0.f, 1.f) : 0.0f;
    if (d->n_run < DEC_N_COOL_BLANK) d->n_run++;
    if (d->n_run < DEC_N_COOL_BLANK) d->cool_on = 0;          /* gözlemci yakınsıyor */
    else if (Ru_hat > DEC_RU_ALARM) d->cool_on = 1;
    else if (Ru_hat < DEC_RU_ALARM_OFF) d->cool_on = 0;
    const float psi_C = d->cool_on ? DEC_PSI_COOL : 0.0f;
    if (T_valid) {                           /* yüzey sıcaklığı artış hızı; sensör geçersizse yeniden başlar */
        if (d->td_on) {
            const float t_prev = d->t_f;
            d->t_f += DEC_TD_A_F * (T_meas - d->t_f);
            d->tdot += DEC_TD_A_R * (60.0f * (d->t_f - t_prev) / BMS_TS - d->tdot);
        } else {
            d->td_on = 1;  d->t_f = T_meas;  d->tdot = 0.0f;
        }
    } else {
        d->td_on = 0;  d->tdot = 0.0f;
    }
    const float psi_D = DEC_PSI_TDOT * clampf((d->tdot - DEC_TDOT_ON) / (DEC_TDOT_FULL - DEC_TDOT_ON), 0.f, 1.f);
    /* açılıştan sonra 120 s Ψ_H/Ψ_A/Ψ_B yok: EKF (SOC, k_R) yakınsıyor (k_R de 120 s donuk, EKF_N_KR_FREEZE) */
    const uint8_t blank = d->n_run < DEC_N_COOL_BLANK;
    /* Ψ_H sağlık (gösterge): kapasite SOH'u ve direnç artışının büyüğü. Yaşlanmış paketin akımını SOP
     * zaten k_R ölçekli R0 ile kısar */
    const float psi_H = blank ? 0.0f : maxf(clampf((DEC_SOH_ON - soh_c) / (DEC_SOH_ON - DEC_SOH_FULL), 0.f, 1.f),
                                            clampf((k_R - DEC_KR_ON) / (DEC_KR_FULL - DEC_KR_ON), 0.f, 1.f));
    /* Ψ_A yaşlanma hızı. Yarım çevrim: |I| > ölü bant iken yön değişince yeni yarım çevrim; m = ortalama SOC,
     * dod = şimdiki salınım ile son tam yarım çevrimin büyüğü */
    const float soc = clampf(SOC_est, 0.f, 1.f), i_abs = fabsf(I_meas);
    float r_inst = 0.0f;
    if (i_abs > DEC_AGE_I_MIN) {
        const int8_t dr = I_meas > 0.0f ? 1 : -1;
        if (dr != d->a_dir) {
            if (d->a_dir != 0) d->a_swing = fabsf(soc - d->a_soc_rev);
            d->a_dir = dr;  d->a_soc_rev = soc;  d->a_sum = 0.0f;  d->a_n = 0;
        }
        d->a_sum += soc;  d->a_n++;
        const float m = d->a_sum / (float)d->a_n;
        const float dod = clampf(maxf(fabsf(soc - d->a_soc_rev), d->a_swing), 0.f, 1.f);
        const float s_rel = stress_S(Tc_hat, m, dod) / STRESS_S_REF;
        r_inst = s_rel * (i_abs / BMS_NP) / DEC_AGE_I_REF;
        d->dmg += s_rel * i_abs * BMS_TS / (2.0f * BMS_NP * CELL_Q_AH * 3600.0f);
    }
    d->r_cyc += DEC_AGE_A * (r_inst - d->r_cyc);
    const float cal_ref = stress_cal_alpha(ocv_clamped(0.5f), 25.0f);
    const float r_cal = stress_cal_alpha(ocv_clamped(soc), Tc_hat) / cal_ref;
    const float psi_cyc = blank ? 0.0f : clampf(log2f(maxf(d->r_cyc, 1e-9f)) / DEC_AGE_LOG2_FULL, 0.f, 1.f);
    const float psi_cal = clampf(log2f(maxf(r_cal / DEC_AGE_CAL_ONSET, 1e-9f)) / DEC_AGE_LOG2_FULL, 0.f, 1.f);
    const float psi_A = blank ? 0.0f : or2(psi_cyc, psi_cal);
    /* Ψ_B grup dengesizliği (gösterge): yalnız düşük akımda güncellenir (IR düşümü karışmasın) */
    if (i_abs < DEC_BAL_I_MAX) {
        if (d->dv_on) d->dv_f += DEC_BAL_A * ((V_max - V_min) - d->dv_f);
        else { d->dv_f = V_max - V_min;  d->dv_on = 1; }
    }
    const float psi_B = (blank || !d->dv_on) ? 0.0f : clampf((d->dv_f - DEC_IMB_ON) / (DEC_IMB_FULL - DEC_IMB_ON), 0.f, 1.f);
    o->psi_i[PSI_T] = psi_T;  o->psi_i[PSI_C] = psi_C;  o->psi_i[PSI_TDOT] = psi_D;
    o->psi_i[PSI_H] = psi_H;  o->psi_i[PSI_A] = psi_A;  o->psi_i[PSI_B] = psi_B;
    float pm = 0.0f, q_all = 1.0f;
    o->dom = 0;
    for (int i = 0; i < PSI_N; i++) {       /* ilk en büyük (Python index() ile aynı) */
        q_all *= 1.0f - o->psi_i[i];
        if (o->psi_i[i] > pm) { pm = o->psi_i[i];  o->dom = (uint8_t)(i + 1); }
    }
    const float psi = 1.0f - (1.0f - psi_T) * (1.0f - psi_C) * (1.0f - psi_D);   /* Ψ_P: deşarjı kısar */
    o->psi_s = 1.0f - q_all;  o->psi_p = psi;  o->tdot = d->tdot;
    o->r_cyc = d->r_cyc;  o->r_cal = r_cal;  o->dmg = d->dmg;

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
        /* termal olay: akım yokken yüzey ortamın üstünde ısınmayı sürdürüyor (bms_config.h DEC_TE_*) */
        d->n_i0 = (fabsf(I_meas) < DEC_TE_I_MAX) ? (d->n_i0 < DEC_TE_WAIT ? d->n_i0 + 1 : DEC_TE_WAIT) : 0;
        const uint8_t rising = CAN_ok && T_valid && T_meas > T_amb && d->tdot > DEC_TE_RATE;
        d->n_te = (rising && d->n_i0 > 0 && (T_anom || d->n_i0 >= DEC_TE_WAIT)) ? d->n_te + 1 : 0;
        if (d->n_te >= DEC_TE_N) d->tevt = 1;
        o->state = ST_SAFE;  o->i_allow_dis = 0.0f;  o->i_allow_chg = 0.0f;  o->psi = 1.0f;
        o->cause = (uint16_t)(128 | (d->tevt ? 2048 : 0));
        return;
    }

    /* ---- açılış sıfır akım kontrolü (A9): kontaktör açık, akım kesin 0 -> ölçülen = ofset ---- */
    const uint8_t zeroing = d->n_zero < DEC_N_ZERO;
    if (zeroing) {                           /* geçersiz örnek varsa sonuç yok (I_valid zaten LIMP) */
        d->i0_sum += I_meas;
        d->i0_ok = d->i0_ok && I_valid;
        if (++d->n_zero == DEC_N_ZERO)
            d->zero_bad = d->i0_ok && fabsf(d->i0_sum / (float)DEC_N_ZERO) > DEC_ZERO_MAX;
    }

    /* ---- veri sayfası sınırları ---- */
    const float i_ds_dis = interp_tab(DEC_T_DIS, DEC_I_DIS, DEC_DS_N, Ts_hat) * BMS_NP;
    const float i_ds_chg = interp_tab(DEC_T_CHG, DEC_I_CHG, DEC_DS_N, Ts_hat) * BMS_NP;
    d->f_ds = band(d->f_ds, minf(i_ds_dis / I_nom_dis, i_ds_chg / I_nom_chg));

    /* ---- gerilim payı (SOP) ---- */
    const float r0 = CELL_R0 * clampf(k_R, 1.0f, DEC_KR_SOP_MAX) * arrhenius(Tc_hat);   /* EKF direnç artışı */
    const float sop_dis = maxf(I_meas, 0.f) + DEC_K_SOP * BMS_NP * (V_min - DEC_V_LIM_DIS) / r0;
    const float sop_chg = maxf(-I_meas, 0.f) + DEC_K_SOP * BMS_NP * (DEC_V_LIM_CHG - V_max) / r0;
    const float r_dc = r0 + CELL_R1 * arrhenius_ea(CELL_EA_R1, Tc_hat) + CELL_R2 * arrhenius_ea(CELL_EA_R2, Tc_hat);
    const float ic = I_meas / BMS_NP;
    const float qs_dis = BMS_NP * (V_min + maxf(ic, 0.f) * r0 - DEC_V_LIM_DIS) / r_dc;
    const float qs_chg = BMS_NP * (DEC_V_LIM_CHG - (V_max - maxf(-ic, 0.f) * r0)) / r_dc;
    d->f_sop_dis = band(d->f_sop_dis, qs_dis / maxf(i_ds_dis, 1e-6f));
    d->f_sop_chg = band(d->f_sop_chg, qs_chg / maxf(i_ds_chg, 1e-6f));

    float tgt_dis = maxf(0.f, minf(i_ds_dis * (1.f - psi), sop_dis));
    /* şarj: koruma (1 - Ψ_P) ve çevrim yaşlanması (1 - Ψ_cyc); bu pay 0,3C tabanının altına inmez */
    const float f_age = minf(1.0f, maxf(1.0f - psi_cyc, DEC_AGE_FLOOR * BMS_NP / maxf(i_ds_chg, 1e-6f)));
    float tgt_chg = maxf(0.f, minf(i_ds_chg * (1.f - psi) * f_age, sop_chg));

    /* ---- LIMP-HOME ---- */
    const uint8_t sensor_bad = !T_valid || !I_valid || fabsf(bias_est) > DEC_BIAS_MAX || d->zero_bad;
    const uint8_t empty = SOC_est < DEC_SOC_EMPTY;
    const uint8_t hot_ss = d->n_hl >= DEC_N_HARD;
    const uint8_t limp = sensor_bad || Tc_hat > DEC_T_LIMP || hot_ss || empty || T_anom;
    if (limp) {
        tgt_dis = minf(minf(DEC_FLOOR * I_nom_dis, i_ds_dis), maxf(sop_dis, 0.f));
        tgt_chg = minf(tgt_chg, DEC_FLOOR * I_nom_chg);
    }
    const uint16_t cause = (psi_T > 0.f || hot_ss ? 1 : 0) | (psi_C > 0.f ? 2 : 0) | (d->f_sop_dis ? 4 : 0)
                         | (d->f_sop_chg ? 8 : 0) | (d->f_ds ? 16 : 0) | (sensor_bad ? 32 : 0)
                         | (empty ? 64 : 0) | (T_anom ? (256 | 1) : 0) | (psi_D > 0.f ? (512 | 1) : 0)
                         | (psi_H > 0.f ? 1024 : 0) | (psi_A > 0.f ? 4096 : 0) | (psi_B > 0.f ? 8192 : 0);

    /* ---- durum + histerezis ---- */
    const uint8_t want = limp ? ST_LIMP : ((cause & ~(1024u | 4096u | 8192u)) ? ST_DERATE : ST_NORMAL);   /* bilgi bitleri */
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
    if (zeroing) { d->i_dis = 0.0f;  d->i_chg = 0.0f; }

    o->state = d->state;  o->i_allow_dis = d->i_dis;  o->i_allow_chg = d->i_chg;
    o->psi = 1.f - d->i_dis / I_nom_dis;  o->cause = cause;
}

/* =================================================================== uygulama */
void bms_init(bms_t *b, float soc0_ekf, float q_scale, float r_scale, float rint_num0, float rint_den0)
{
    ekf_init(&b->ekf, soc0_ekf, q_scale, r_scale);
    thermal_init(&b->th);
    tanom_init(&b->ta);
    diag_init(&b->diag);
    rint_init(&b->rint, rint_num0, rint_den0);
    decision_init(&b->dec);
    b->soh_c = 1.0f;
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
    ekf_zero(&b->ekf, in->I);
    o->soc = b->ekf.x[0];
    o->bias = b->ekf.x[3];

    const float g_prev = b->th.x[2];
    thermal_step(&b->th, in->I, in->V, o->soc, in->T, in->T_amb, o->t_valid);
    /* termal anomali: g donar (gizli ısı soğutma arızası sanılmasın), Tc ölçüme bağlı kararlı
     * hal sınırının altına inmez (iç kaynakta çekirdek yüzeyden sıcak) */
    o->t_anom = tanom_step(&b->ta, in->I, in->V, o->soc, in->T, in->T_amb, b->th.x[0], o->t_valid);
    if (o->t_anom) {
        b->th.x[2] = g_prev;
        if (o->t_valid) b->th.x[0] = maxf(b->th.x[0], in->T + DEC_K_SS * (in->T - in->T_amb));
    }
    o->tc = b->th.x[0];  o->ts = b->th.x[1];  o->ru = 1.0f / b->th.x[2];  o->kr = b->ekf.x[4];

    o->rr = rint_step(&b->rint, in->I, in->V, o->tc, o->soc, o->i_valid);

    decision_step(&b->dec, in->I, in->V_min, in->V_max, o->tc, o->ts, o->ru, o->soc, o->bias,
                  o->t_valid, o->i_valid, in->can_ok, in->T, in->T_amb, o->t_anom, o->kr, b->soh_c, &o->dec);
}

void bms_set_capacity(bms_t *b, float soh) { b->ekf.q_ah = soh * SOH_Q_NOM;  b->soh_c = soh; }
void bms_set_kr(bms_t *b, float k_r) { b->ekf.x[4] = clampf(k_r, EKF_KR_MIN, EKF_KR_MAX); }

/* =================================================================== kapasite SOH'u (park-park arası)
 * Dinlenme 20 dk sürünce bir nokta alınır: son 10 s'nin ortalama hücre gerilimi (gevşeme bitmiş) OCV
 * tablosunun tersinden SOC verir; aynı pencerenin akım ortalaması sensör ofsetini yeniler. İlk geçerli nokta
 * çapa olur; sonraki noktada |dSOC| yeterliyse kapasite = çapadan beri sayılan Ah / dSOC. Kazanç ölçümün
 * kalitesine (|dSOC|) ve ölçüm sayısına bağlı (ilk ölçüm %100 önselini yarı yola taşır). EKF'den bağımsız.
 * Sıra ve denklemler twin/soh_capacity.py ile aynı (float32). */
float soc_from_ocv(float ocv_cell, float *slope)
{   /* tablonun tersi, doğrusal ara değer; slope = bulunan aralığın dOCV/dSOC'si [V] */
    int k = 1;
    while (k < SOH_OCV_N - 1 && ocv_cell > SOH_OCV_V[k]) k++;
    const float v0 = SOH_OCV_V[k - 1], v1 = SOH_OCV_V[k], s0 = SOH_OCV_SOC[k - 1], s1 = SOH_OCV_SOC[k];
    *slope = (v1 - v0) / (s1 - s0);
    return clampf(s0 + (ocv_cell - v0) * (s1 - s0) / (v1 - v0), 0.0f, 1.0f);
}

void soh_init(soh_t *s, const soh_rec_t *rec)
{
    memset(s, 0, sizeof(*s));
    if (rec) s->r = *rec;
    else { s->r.soh_c = 1.0f;  s->r.soh_disp = 1.0f;  s->r.k_r = 1.0f; }
    s->ah_saved = s->r.ah_total;
    s->kr_saved = s->r.k_r;
}

void soh_reset_anchor(soh_t *s)
{
    if (s->r.anchor) s->persist = 1u;            /* kayıttaki çapa da düşsün */
    s->r.anchor = 0u;  s->n_rest = 0;  s->ocv_sum = s->i_sum = 0.0f;
}

static void soh_point(soh_t *s, float T_meas, float k_r)
{
    float slope;
    const float soc = soc_from_ocv(s->ocv_sum / (float)SOH_N_AVG, &slope);
    s->b0 = s->i_sum / (float)SOH_N_AVG;         /* dinlenmede gerçek akım ~0: ölçülen ortalama = ofset */
    const uint8_t ok = T_meas >= SOH_T_MIN && T_meas <= SOH_T_MAX && slope >= SOH_SLOPE_MIN;
    s->last_soc = soc;
    if (!ok) {
        s->event = SOH_EV_POINT;                 /* geçersiz uç: çapa korunur, Ah sayımı sürer */
    } else if (!s->r.anchor) {
        s->r.anchor = 1u;  s->r.soc_a = soc;  s->r.ah_a = 0.0f;  s->n_anchor = 0;
        s->persist = 1u;
        s->event = SOH_EV_ANCHOR;
    } else {
        const float dsoc = s->r.soc_a - soc;
        if (fabsf(dsoc) < SOH_DSOC_MIN) {
            s->event = SOH_EV_SMALL;             /* çapa korunur: kısa park ölçümü bölmez */
        } else {
            const float ratio = s->r.ah_a / dsoc / SOH_Q_NOM;
            s->last_ratio = ratio;
            if (ratio >= SOH_R_MIN && ratio <= SOH_R_MAX) {
                const float w = minf(1.0f, fabsf(dsoc) / SOH_DSOC_W);
                const float n = (float)(s->r.n_meas < (uint32_t)SOH_N_MAX - 1u ? s->r.n_meas
                                                                              : (uint32_t)SOH_N_MAX - 1u) + 1.0f;
                s->r.soh_c += w / (n + w) * (ratio - s->r.soh_c);
                s->r.soh_disp = minf(1.0f, s->r.soh_c < s->r.soh_disp ? s->r.soh_c
                                                     : minf(s->r.soh_c, s->r.soh_disp + SOH_DISP_UP));
                s->r.n_meas++;
                s->event = SOH_EV_ACCEPT;
            } else {
                s->event = SOH_EV_RANGE;
            }
            s->r.soc_a = soc;  s->r.ah_a = 0.0f;  s->n_anchor = 0;   /* bu nokta yeni çapa */
            s->persist = 1u;
        }
    }
    s->r.k_r = k_r;
    if (fabsf(s->r.ah_total - s->ah_saved) >= SOH_AH_SAVE || fabsf(s->r.k_r - s->kr_saved) >= SOH_KR_SAVE)
        s->persist = 1u;
}

void soh_step(soh_t *s, const bms_in_t *in, const bms_out_t *o)
{
    s->event = SOH_EV_NONE;
    if (s->n_zero < DEC_N_ZERO) {                /* açılış: kontaktör açık, I_allow = 0 (A9) */
        s->b0 += in->I;
        if (++s->n_zero == DEC_N_ZERO) s->b0 /= (float)DEC_N_ZERO;
        return;
    }
    const float i = in->I - s->b0;
    const uint8_t valid = o->t_valid && o->i_valid && in->can_ok;
    s->r.ah_total += fabsf(i) * (BMS_TS / 3600.0f);
    if (!valid) soh_reset_anchor(s);
    if (s->r.anchor) {
        s->r.ah_a += i * (BMS_TS / 3600.0f);
        if (++s->n_anchor > SOH_N_ANCHOR) soh_reset_anchor(s);
    }
    s->i_f += SOH_I_FILT * (i - s->i_f);
    if (valid && fabsf(s->i_f) < SOH_I_REST) {
        if (s->n_rest < SOH_N_REST) {
            s->n_rest++;
            if (s->n_rest > SOH_N_REST - SOH_N_AVG) {
                s->ocv_sum += in->V / (float)BMS_NS;
                s->i_sum += in->I;
            }
            if (s->n_rest == SOH_N_REST) soh_point(s, in->T, o->kr);
        }
    } else {
        s->n_rest = 0;  s->ocv_sum = s->i_sum = 0.0f;
    }
}
