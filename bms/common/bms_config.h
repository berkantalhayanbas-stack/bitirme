/* bms_config.h -- BMS ECU sabitleri (19S12P LG INR21700 M50T).
 * Kaynak: simulink/bms_param_struct.m ve twin/ altındaki Python dosyalarıyla birebir aynı değerler.
 * Platformdan bağımsız: STM32G431 (Cortex-M4F, float32 FPU) ve PC (SIL testi).
 */
#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

#define BMS_TS          0.1f        /* örnekleme zamanı [s] (CAN 0x101 periyodu 100 ms) */
#define BMS_NS          19
#define BMS_NP          12

/* ---- hücre (25 °C, BoL) ---- */
#define CELL_Q_AH       4.85f
#define CELL_R0         0.0264f
#define CELL_R1         0.0093f
#define CELL_C1         1075.0f
#define CELL_R2         0.012f
#define CELL_C2         15000.0f
#define CELL_EA_R0      20000.0f    /* [J/mol] */
#define RGAS            8.314f
#define CELL_CC         64.0f       /* [J/K] */
#define CELL_CS         5.0f
#define CELL_RC         1.8f        /* [K/W] */
#define CELL_RU         9.0f

/* OCV polinomu (hücre, 7. derece, en yüksek dereceden başlayarak) */
#define OCV_POLY_N      8
static const float OCV_POLY[OCV_POLY_N] = {
    103.853146128f, -379.901612457f, 567.288148072f, -450.895141102f,
    208.968987223f, -57.5219165548f, 9.68100441089f, 2.70638848517f };

/* ---- SOC EKF, x = [SOC V1 V2 b_I] ---- */
#define EKF_P0          { 0.1f, 0.01f, 0.01f, 1.0f }
#define EKF_Q           { 1e-10f, 1e-6f, 1e-6f, 1e-8f }
#define EKF_R           (0.25f * 0.25f)
#define EKF_GATE        10.0f       /* normalize inovasyon kapısı [sigma] */
#define EKF_N_REJ       5           /* art arda red sınırı -> kapı açılır */

/* ---- sıcaklık gözlemcisi, x = [Tc Ts g] ---- */
#define TH_P0           { 1.0f, 0.1f, (1.0f / (CELL_RU * CELL_RU)) }
#define TH_Q            { 1e-4f, 1e-4f, 1e-9f }
#define TH_R            (0.3f * 0.3f + 0.1f * 0.1f / 12.0f)
#define TH_G_MIN        (0.2f / CELL_RU)

/* ---- sensör teşhisi ---- */
#define DIAG_T_MIN      (-30.0f)
#define DIAG_T_MAX      85.0f
#define DIAG_T_RATE     2.0f        /* [°C / örnek] */
#define DIAG_N_FROZEN   50          /* 5 s */

/* ---- çevrimiçi direnç kestiricisi ---- */
#define RINT_DI_MIN     20.0f       /* [A] */
#define RINT_SOC_MIN    0.2f
#define RINT_LAM        0.999f

/* ---- karar mekanizması ---- */
#define DEC_T_DER_ON    45.0f
#define DEC_T_DER_OFF   43.0f
#define DEC_T_DER_FULL  55.0f
#define DEC_PSI_COOL    0.3f
#define DEC_RU_ALARM    18.0f       /* [K/W] soğutma alarmı açılır (2 x nominal R_u) */
#define DEC_RU_ALARM_OFF 15.0f      /* [K/W] altına inince kalkar (histerezis; kestirim eşik
                                       çevresinde yakınsarken alarm açılıp kapanıyordu) */
#define DEC_N_COOL_BLANK 1200       /* 120 s: açılıştan sonra soğutma alarmı yok. Gözlemci sıcak ve
                                       yük altındaki pakette başlarsa (Tc > Ts iken Tc = Ts varsayar)
                                       R_u kestirimi geçici olarak 30 K/W'ı aşıyor; taramada son sahte
                                       alarm < 60 s (twin/test_decision_unit.py U13, 2x pay). Sıcaklık
                                       koruması bu sürede de çalışır. */
#define DEC_V_LIM_DIS   2.90f
#define DEC_V_LIM_CHG   4.18f
#define DEC_K_SOP       0.3f
#define DEC_R_RC        (0.0093f + 0.012f)
#define DEC_BAND_ON     0.90f
#define DEC_BAND_OFF    1.00f
#define DEC_FLOOR       0.25f
#define DEC_T_LIMP      55.0f
#define DEC_T_SAFE      60.0f
#define DEC_SOC_EMPTY   0.03f
#define DEC_V_SAFE_LO   2.50f
#define DEC_V_SAFE_HI   4.25f
#define DEC_N_HARD      10          /* 1 s */
#define DEC_BIAS_MAX    5.0f
#define DEC_N_EXIT      100         /* 10 s */
#define DEC_RAMP_UP     10.0f       /* [A/s] */
/* veri sayfası sürekli akım sınırları [A/hücre], 2 °C geçişli parçalı doğrusal tablo */
#define DEC_DS_N        6
static const float DEC_T_DIS[DEC_DS_N] = { -20.f, -18.f, 8.f, 10.f, 53.f, 55.f };
static const float DEC_I_DIS[DEC_DS_N] = { 0.f, 2.425f, 2.425f, 7.275f, 7.275f, 0.f };
static const float DEC_T_CHG[DEC_DS_N] = { 0.f, 2.f, 23.f, 25.f, 43.f, 45.f };
static const float DEC_I_CHG[DEC_DS_N] = { 0.f, 1.455f, 1.455f, 3.395f, 3.395f, 0.f };

/* ---------------- pasif dengeleme (bms_bal.c, HIL adım 8). Kaynak: arac_bms_analizi.md bölüm 4-5
 * (MathWorks BMS referansı: hedef fark 10 mV, açma bekleme 60 s; sürüşte kapalı). Henüz
 * simulink/ ve twin/ tarafında karşılığı yok. */
#define BAL_DV_ON       0.010f      /* V_i - V_min > 10 mV -> grubun direnci açılır [V] */
#define BAL_DV_OFF      0.005f      /* < 5 mV -> kapanır (histerezis) */
#define BAL_I_DRIVE     2.0f        /* I > 2 A deşarj = sürüş -> dengeleme hemen kapanır [A] */
#define BAL_T_ON        60.0f       /* koşullar bu kadar kesintisiz sağlanınca başlar [s] */
#define BAL_V_MIN       3.40f       /* en düşük grup bunun altındaysa dengeleme yok [V] */
#define BAL_T_MAX       45.0f       /* Tc_hat üstünde dengeleme yok (direnç ısısı) [°C] */
#define BAL_FILT        0.1f        /* grup gerilimi alçak geçiren katsayısı (100 ms'de; tau ~1 s) */

#endif
