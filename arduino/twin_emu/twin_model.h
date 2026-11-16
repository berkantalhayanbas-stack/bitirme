/* twin_model.h -- 19S12P paket elektro-termal ikizinin C portu (C:\bitirme\twin\battery_model.py).
 *
 * Elektriksel: hücre başına 2-RC Thevenin, R0(T, SOC) Arrhenius + düşük SOC artışı, OCV(SOC) tablosu
 * Termal     : 2 düğümlü (çekirdek-yüzey) toplu model, Bernardi tersinmez ısı, açık Euler
 * İşaret     : I > 0 deşarj, I < 0 şarj
 *
 * Parametreler bms/common/bms_config.h, simulink/bms_param_struct.m ve twin/battery_model.py ile
 * AYNI olmalı; biri değişirse burası da değişmeli.
 *
 * Hücre grupları: TW_NG adet seri grup simüle edilir; paket gerilimi = (19 / TW_NG) x toplam.
 * TW_NG = 1 -> tüm gruplar özdeş (Python PackTwin). HIL adım 8'de TW_NG = 19 yapılıp her gruba
 * kq (kapasite) / kr (direnç) dağılımı verilecek; grup başına 20 bayt RAM (19 grup ~380 B).
 * Termal model ortalama hücre için tektir (gruplar arası sıcaklık farkı modellenmez).
 */
#ifndef TWIN_MODEL_H
#define TWIN_MODEL_H
#include <stdint.h>

#define TW_NG       1           /* simüle edilen seri grup sayısı (adım 8: 19) */
#define TW_NS       19
#define TW_NP       12

typedef struct {
    float soc, v1, v2;          /* durum: SOC, RC1 ve RC2 gerilimleri (hücre) */
    float kq, kr;               /* kapasite ve direnç çarpanı (1 = nominal) */
    float v;                    /* son adımdaki hücre uç gerilimi */
} tw_group_t;

typedef struct {
    tw_group_t g[TW_NG];
    float tc, ts;               /* ortalama hücre çekirdek / yüzey sıcaklığı [°C] */
    float t_amb;                /* ortam [°C], her adımdan önce dışarıdan yazılabilir */
    float a1, a2, dt;           /* RC ayrıklaştırma katsayıları (ZOH) */
    float v_pack, v_min, v_max; /* çıktı: paket gerilimi, en düşük / en yüksek grup gerilimi */
    float soc_avg;              /* çıktı: grup SOC ortalaması */
} twin_t;

void  twin_init(twin_t *tw, float soc0, float t_amb, float dt);
void  twin_set_soc(twin_t *tw, float soc);
void  twin_step(twin_t *tw, float i_pack);
float twin_ocv(float soc);      /* hücre OCV [V] */

#endif
