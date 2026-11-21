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
#define CELL_EA_R0      6200.0f     /* [J/mol] Arrhenius: Kirkaldy 2024 ham döngü verisi, 8 hücre */
#define CELL_EA_R1      17500.0f    /*   15-40 °C, 5 A basamağı (bitirme veri/ea_kirkaldy.py): R0 =   */
#define CELL_EA_R2      14300.0f    /*   0,05 s ohmik, R1 / R2 = 10 s / 30 s polarizasyonu            */
#define RGAS            8.314f
#define CELL_CC         64.0f       /* [J/K] */
#define CELL_CS         5.0f
#define CELL_RC         1.8f        /* [K/W] */
#define CELL_RU         9.0f

/* OCV polinomu (hücre, 7. derece, en yüksek dereceden başlayarak; SOC >= 0,05'te Kirkaldy vd. 2024
 * LG M50T GITT OCV tablosuna uydurma, en büyük hata 11,6 mV; bms_param_struct.m ile aynı) */
#define OCV_POLY_N      8
static const float OCV_POLY[OCV_POLY_N] = {
    60.3790948991f, -208.990896881f, 295.06003585f, -224.987541467f,
    104.426799217f, -30.805913718f, 6.20417948038f, 2.89779228523f };

/* ---- SOC EKF, x = [SOC V1 V2 b_I] ---- */
#define EKF_P0          { 0.1f, 0.01f, 0.01f, 1.0f, 0.01f }  /* son: k_R ±%10 */
#define EKF_Q           { 1e-10f, 1e-6f, 1e-6f, 1e-8f, 1e-8f }
#define EKF_KR_MIN      0.5f        /* k_R = gerçek R0 / nominal R0 sınırları */
#define EKF_KR_MAX      2.0f
#define EKF_P_B0        0.05f       /* [A^2] sıfır akım kalibrasyonundan sonra kayma belirsizliği
                                       (10 örnek ortalaması: si^2/10 ~ 0,025) */
#define EKF_R           (0.25f * 0.25f)
#define EKF_GATE        10.0f       /* normalize inovasyon kapısı [sigma] */
#define EKF_N_REJ       5           /* art arda red sınırı -> kapı açılır */

/* ---- sıcaklık gözlemcisi, x = [Tc Ts g] ---- */
#define TH_P0           { 1.0f, 0.1f, (1.0f / (CELL_RU * CELL_RU)) }
#define TH_Q            { 1e-4f, 1e-4f, 1e-9f }
#define TH_R            (0.3f * 0.3f + 0.1f * 0.1f / 12.0f)
#define TH_G_MIN        (0.2f / CELL_RU)

/* ---- termal anomali dedektörü (açıklanamayan ısı; HIL A2 bulgusu, 2026-11-21) ----
 * Gözlemci yalnız Ts'yi ölçer ve ısıyı I^2R + ortamla açıklar; gizli bir kaynak (dış ısı, iç kısa
 * devre başlangıcı) g'yi alt sınıra iter, Tc düşük kalır. "Adyabatik zarf": bilinen ısının üst sınırı
 * (TA_Q_K x + TA_Q_OFF) ve SOĞUTMA YOK (Ta > Ts ise en güçlü ısıtma g = TA_G_MAX) ile Ts'nin
 * çıkabileceği en yüksek değer. İki kaydırmalı pencere (TA_WIN, yarım periyot kayık), ölçümden
 * yeniden başlar. Süzülmüş ölçüm zarfı TA_R_TH aşar, TA_N örnek sürerse anomali (kilitlenir). */
#define TA_WIN          1200        /* 120 s pencere [örnek] */
#define TA_AGE_MIN      100         /* zarf en az 10 s yaşlı olmalı [örnek] */
#define TA_R_TH         1.5f        /* [°C] */
#define TA_N            10          /* 1 s */
#define TA_ALPHA        0.1f        /* ölçüm süzgeci (~1 s) */
#define TA_Q_K          1.5f        /* bilinen ısı payı (R0/ısı kapasitesi belirsizliği) */
#define TA_Q_OFF        0.2f        /* [W/hücre] */
#define TA_TC0          1.0f        /* [°C] zarf başlangıç payları: Tc = max(Tc_hat, Tm) + TA_TC0 */
#define TA_TS0          0.5f        /*       Ts = Tm + TA_TS0 */
#define TA_G_MAX        (4.0f / CELL_RU)

/* ---- sensör teşhisi ---- */
#define DIAG_T_MIN      (-30.0f)
#define DIAG_T_MAX      85.0f
#define DIAG_T_RATE     2.0f        /* [°C] ölçümün hızlı süzgeçten farkı, DIAG_N_RATE örnek üst üste.
                                       Tek örneklik |dT| > 2 °C kuralı 2 kat gürültüde (0,6 °C) sahte
                                       arıza verip sürüşü LIMP'e kilitliyordu (A3/A8). */
#define DIAG_T_ALPHA    0.28f       /* 1 - exp(-0,1 / 0,3 s) */
#define DIAG_N_RATE     3
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
#define DEC_BAND_ON     0.90f
#define DEC_BAND_OFF    1.00f
#define DEC_FLOOR       0.25f
#define DEC_T_LIMP      55.0f
#define DEC_T_SAFE      60.0f
#define DEC_K_SS        (CELL_RC / CELL_RU)  /* ölçüme bağlı çekirdek sınırı T_ss = Tm + k (Tm - Ta):
                                       kararlı halde Tc - Ts = Rc/Ru (Ts - Ta), ısının kaynağından
                                       bağımsız. Gözlemci modellenmemiş ısıyı (dış kaynak, iç kısa
                                       devre başlangıcı) görmez; T_ss > 55 / 60 °C (1 s) LIMP / SAFE.
                                       HIL 3. bulgu: A2'de SAFE gerçek çekirdek 66 °C'de geliyordu. */
#define DEC_SOC_EMPTY   0.03f
#define DEC_V_SAFE_LO   2.50f
#define DEC_V_SAFE_HI   4.25f
#define DEC_N_HARD      10          /* 1 s */
#define DEC_BIAS_MAX    8.0f        /* [A] 5 A: direnç uyumsuzluğunda sahte sensör LIMP'i (A3/A7) */
#define DEC_N_EXIT      100         /* 10 s */
#define DEC_RAMP_UP     10.0f       /* [A/s] */
#define DEC_N_ZERO      10          /* 1 s: açılışta kontaktör açık, I_allow = 0 (A9 sıfır akım kontrolü) */
#define DEC_ZERO_MAX    3.0f        /* [A] |ölçülen ortalama| üstü: akım sensörü arızası (LIMP);
                                       altı: EKF kayma durumu bununla başlar (300 A sensörün %1'i) */
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
