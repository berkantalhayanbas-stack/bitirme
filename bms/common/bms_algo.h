/* bms_algo.h -- BMS kestirim ve karar algoritmaları (STM32 + PC SIL ortak kodu).
 * Her modül: durum yapısı + init + step. Dinamik bellek yok, float32.
 * Python referansları: twin/ekf.py, thermal_observer.py, sensor_faults.py,
 *                      rint_estimator.py, decision.py; ML: ml/soh_tree.c
 */
#ifndef BMS_ALGO_H
#define BMS_ALGO_H
#include <stdint.h>

/* ------------------------------------------------------------ SOC EKF */
typedef struct {
    float x[4];          /* SOC, V1, V2, b_I */
    float P[4][4];
    float r0_ref, r1, r2, a1, a2, q_ah;
    int   n_rej;
} ekf_t;
void  ekf_init(ekf_t *e, float soc0, float q_scale, float r_scale);
/* T_use: R0(T) düzeltmesi için sıcaklık; rejected: inovasyon kapısı ölçümü reddetti */
void  ekf_step(ekf_t *e, float i_meas, float v_meas, float T_use, float *innov, uint8_t *rejected);

/* ------------------------------------------------------------ sıcaklık gözlemcisi */
typedef struct {
    float x[3];          /* Tc, Ts, g = 1/Ru */
    float P[3][3];
    uint8_t started;
} thermal_t;
void thermal_init(thermal_t *t);
void thermal_start(thermal_t *t, float T_meas);
void thermal_step(thermal_t *t, float i_meas, float v_meas, float soc_est, float T_meas,
                  float T_amb, uint8_t T_valid);

/* ------------------------------------------------------------ sensör teşhisi */
typedef struct {
    float t_prev, i_prev;
    int   n_t, n_i;
    uint8_t first, t_valid, i_valid;
} diag_t;
void diag_init(diag_t *d);
void diag_step(diag_t *d, float i_meas, float T_meas);

/* ------------------------------------------------------------ direnç kestiricisi */
typedef struct {
    float num, den, i_prev, v_prev;
    int   n;
    uint8_t first;
} rint_t;
void  rint_init(rint_t *r, float num0, float den0);
float rint_step(rint_t *r, float i_meas, float v_meas, float Tc_hat, float soc_est, uint8_t I_valid);

/* ------------------------------------------------------------ karar mekanizması */
enum { ST_NORMAL = 1, ST_DERATE = 2, ST_LIMP = 3, ST_SAFE = 4 };
typedef struct {
    uint8_t state, therm_on, f_ds, f_sop_dis, f_sop_chg;
    int   n_ok, n_lo, n_hi;
    float i_dis, i_chg;
} decision_t;
typedef struct {
    uint8_t  state;
    float    i_allow_dis, i_allow_chg, psi;
    uint16_t cause;
} decision_out_t;
void decision_init(decision_t *d);
void decision_step(decision_t *d, float I_meas, float V_min, float V_max, float Tc_hat,
                   float Ts_hat, float Ru_hat, float SOC_est, float bias_est,
                   uint8_t T_valid, uint8_t I_valid, uint8_t CAN_ok, decision_out_t *o);

/* ------------------------------------------------------------ ML: SOH ağacı */
float soh_tree(float efc, float days, float T, float m, float dod, float rr);

/* ------------------------------------------------------------ uygulama (tek adım) */
typedef struct {            /* CAN'dan gelen ölçümler (0x101 / 0x102), fiziksel birim */
    float I, V, T, V_min, V_max, T_amb;
    uint8_t can_ok;
} bms_in_t;
typedef struct {
    float soc, bias, innov;
    uint8_t rejected;
    float tc, ts, ru;
    uint8_t t_valid, i_valid;
    float rr;
    decision_out_t dec;
} bms_out_t;
typedef struct {
    ekf_t ekf; thermal_t th; diag_t diag; rint_t rint; decision_t dec;
} bms_t;
void bms_init(bms_t *b, float soc0_ekf, float q_scale, float r_scale, float rint_num0, float rint_den0);
void bms_step(bms_t *b, const bms_in_t *in, bms_out_t *out);

#endif
