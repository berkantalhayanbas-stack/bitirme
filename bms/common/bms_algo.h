/* bms_algo.h -- BMS kestirim ve karar algoritmaları (STM32 + PC SIL ortak kodu).
 * Her modül: durum yapısı + init + step. Dinamik bellek yok, float32.
 * Python referansları: twin/ekf.py, thermal_observer.py, sensor_faults.py,
 *                      rint_estimator.py, decision.py; ML: ml/soh_tree.c
 */
#ifndef BMS_ALGO_H
#define BMS_ALGO_H
#include <stdint.h>

/* ------------------------------------------------------------ SOC EKF */
#define EKF_N 5
typedef struct {
    float x[EKF_N];      /* SOC, V1, V2, b_I, k_R (R0 ölçeği; A7) */
    float P[EKF_N][EKF_N];
    float r0_ref, r1, r2, a1, a2, q_ah;
    int   n_rej;
    int   n_zero;        /* açılış sıfır akım kalibrasyonu (A9) */
    int   n_kr;          /* açılıştan beri adım (k_R dondurma, EKF_N_KR_FREEZE) */
    float i0_sum;
} ekf_t;
void  ekf_init(ekf_t *e, float soc0, float q_scale, float r_scale);
/* T_use: R0(T) düzeltmesi için sıcaklık; rejected: inovasyon kapısı ölçümü reddetti */
void  ekf_step(ekf_t *e, float i_meas, float v_meas, float T_use, float *innov, uint8_t *rejected);
/* ilk DEC_N_ZERO adımda (karar I_allow = 0 tutar) ölçülen akımı toplar, sonunda kayma durumunu
 * ortalamayla başlatır (büyükse karar ayrıca LIMP verir; bilinen ofset yine telafi edilir).
 * ekf_step'ten sonra çağrılır. */
void  ekf_zero(ekf_t *e, float i_meas);

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

/* ------------------------------------------------------------ termal anomali (açıklanamayan ısı) */
typedef struct {
    float tc[2], ts[2];  /* iki adyabatik zarf */
    int   age[2], k, n;
    float t_f;           /* süzülmüş ölçüm */
    uint8_t started, flag;   /* flag kilitlenir (yeniden başlatmaya kadar) */
} tanom_t;
void    tanom_init(tanom_t *a);
uint8_t tanom_step(tanom_t *a, float i_meas, float v_meas, float soc_est, float T_meas, float T_amb,
                   float Tc_hat, uint8_t T_valid);

/* ------------------------------------------------------------ sensör teşhisi */
typedef struct {
    float t_prev, i_prev, t_lp;     /* t_lp: değişim hızı teşhisi için hızlı süzgeç */
    int   n_t, n_i, n_rate;
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
    uint8_t state, therm_on, cool_on, f_ds, f_sop_dis, f_sop_chg;
    int   n_ok, n_lo, n_hi, n_run;   /* n_run: açılıştan beri örnek (soğutma alarmı beklemesi) */
    int   n_hl, n_hs;                /* ölçüme bağlı çekirdek sınırı: LIMP / SAFE süre sayaçları */
    int   n_zero;                    /* açılış sıfır akım kontrolü (A9): örnek sayısı */
    float i0_sum;
    uint8_t i0_ok, zero_bad;
    float i_dis, i_chg;
    uint8_t td_on;                   /* Ψ_Tdot süzgeci başladı mı */
    float t_f, tdot;                 /* süzülmüş yüzey T, artış hızı [°C/dk] */
    int   n_i0, n_te;                /* termal olay: SAFE'te akımsız örnek, koşul süresi */
    uint8_t tevt;                    /* termal olay (kilitlenir) */
    int8_t  a_dir;                   /* Ψ_A yarım çevrim: +1 deşarj, -1 şarj, 0 henüz yok */
    int     a_n;
    float   a_soc_rev, a_sum, a_swing;   /* yön değişimindeki SOC, SOC toplamı, son yarım çevrimin derinliği */
    float   r_cyc, dmg;              /* süzülmüş çevrim hızı oranı, birikimli hasar (referans EFC) */
    uint8_t dv_on;
    float   dv_f;                    /* Ψ_B süzgeci [V] */
} decision_t;
enum { PSI_T = 0, PSI_C, PSI_TDOT, PSI_H, PSI_A, PSI_B, PSI_N };   /* baskın kriter kodu = indeks + 1 (0 yok) */
typedef struct {
    uint8_t  state;
    float    i_allow_dis, i_allow_chg, psi;   /* psi: toplam kısıtlama 1 - I_allow/I_nom (0x202) */
    uint16_t cause;
    float    psi_i[PSI_N], psi_s, psi_p, tdot;  /* alt indeksler, Ψ = VEYA(hepsi), Ψ_P = VEYA(T, C, Tdot), dT/dt */
    float    r_cyc, r_cal, dmg;               /* yaşlanma hızı oranları (çevrim, takvim), birikimli hasar */
    uint8_t  dom;                             /* baskın kriter: 0 yok, 1 T, 2 C, 3 Tdot, 4 H, 5 A, 6 B */
} decision_out_t;
void decision_init(decision_t *d);
void decision_step(decision_t *d, float I_meas, float V_min, float V_max, float Tc_hat,
                   float Ts_hat, float Ru_hat, float SOC_est, float bias_est,
                   uint8_t T_valid, uint8_t I_valid, uint8_t CAN_ok, float T_meas, float T_amb,
                   uint8_t T_anom, float k_R, float soh_c, decision_out_t *o);
/* cause bitleri: 1 sıcaklık, 2 soğutma, 4/8 gerilim payı, 16 veri sayfası, 32 sensör, 64 boş,
 * 128 sert sınır, 256 termal anomali, 512 sıcaklık artış hızı, 1024 sağlık (Ψ_H), 2048 termal olay,
 * 4096 yaşlanma hızı (Ψ_A), 8192 grup dengesizliği (Ψ_B) (1024/4096/8192 bilgi: DERATE vermez)
 * (yalnız SAFE'te: 128 | 2048) (0x201'de cause 8 bit: anomali ve artış hızı bit 1'i de kurar; direnç
 * artışının ve termal olayın 8 bitlik karşılığı yok, 0x201 SAFE'te 128 gönderir) */

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
    float tc, ts, ru, kr;            /* kr: EKF R0 ölçeği k_R */
    uint8_t t_valid, i_valid, t_anom;
    float rr;
    decision_out_t dec;
} bms_out_t;
typedef struct {
    ekf_t ekf; thermal_t th; tanom_t ta; diag_t diag; rint_t rint; decision_t dec;
    float soh_c;            /* kapasite SOH'u (bms_set_capacity): Ψ_H girdisi */
} bms_t;
void bms_init(bms_t *b, float soc0_ekf, float q_scale, float r_scale, float rint_num0, float rint_den0);
void bms_step(bms_t *b, const bms_in_t *in, bms_out_t *out);
/* öğrenilmiş değerler (kalıcı kayıt / kapasite SOH'u) EKF'ye: kapasite = soh x nominal, R0 ölçeği */
void bms_set_capacity(bms_t *b, float soh);
void bms_set_kr(bms_t *b, float k_r);

/* ------------------------------------------------------------ kapasite SOH'u (park-park arası)
 * bms_step'ten sonra çağrılır (PIL/SIL zinciri değişmez); I/O yapmaz, kayıt gerekince persist = 1.
 * Python referansı: twin/soh_capacity.py */
enum { SOH_EV_NONE = 0, SOH_EV_ANCHOR, SOH_EV_ACCEPT, SOH_EV_SMALL, SOH_EV_RANGE, SOH_EV_POINT };
typedef struct {            /* kalıcı kısım (flash kaydı): çapa da kalıcı, ölçüm kapatıp açmayı aşar
                             * (park -> kontak kapalı -> sabah açılış; kapalıyken akım akmaz) */
    float soh_c, soh_disp, k_r, ah_total;
    float soc_a, ah_a;          /* çapa: OCV SOC'si, çapadan beri net Ah */
    uint32_t n_meas, anchor;
} soh_rec_t;
typedef struct {
    soh_rec_t r;
    float b0;                       /* akım sensörü ofseti: açılışta (kontaktör açık) ve her dinlenme
                                     * noktasında ölçülür. EKF'nin b_I'si sürüşte kayıyor (A7), kullanılmaz */
    int   n_zero;                   /* açılış ofset ortalaması için örnek (DEC_N_ZERO) */
    float i_f;                      /* süzülmüş I - b0 (işaretli; dinlenme |i_f| < SOH_I_REST) */
    int   n_rest;                   /* dinlenme süresi [örnek] */
    float ocv_sum, i_sum;           /* son SOH_N_AVG örneğin hücre OCV / akım toplamı */
    int   n_anchor;                 /* çapanın bu açılıştaki yaşı [örnek] (r.anchor / r.soc_a / r.ah_a) */
    float ah_saved, kr_saved;       /* son yazılan kayıttaki değerler */
    uint8_t persist, event;         /* kayıt iste; son olay (SOH_EV_*) */
    float last_ratio, last_soc;     /* son ölçülen kapasite oranı, son dinlenme SOC'si */
} soh_t;
void  soh_init(soh_t *s, const soh_rec_t *rec);   /* rec = 0: yeni paket (%100) */
void  soh_reset_anchor(soh_t *s);                  /* SOC ayarlandı / dış müdahale: ölçüm zinciri kırılır */
/* her adımda: ölçüm (I, V, T, can_ok) ve bms çıktısı (T/I geçerliliği, k_R). EKF'ye bağlı değil:
 * dinlenme gerilimi tablodan SOC, ofset kendi ölçümü (EKF'nin kapasitesi ölçümü etkilemez) */
void  soh_step(soh_t *s, const bms_in_t *in, const bms_out_t *o);
float soc_from_ocv(float ocv_cell, float *slope); /* OCV tablosunun tersi; slope: dOCV/dSOC [V] */

#endif
