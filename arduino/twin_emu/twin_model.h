/* twin_model.h -- 19S12P paket elektro-termal ikizinin C portu (C:\bitirme\twin\battery_model.py).
 *
 * Elektriksel: hücre başına 2-RC Thevenin, R0(T, SOC) Arrhenius + düşük SOC artışı, OCV(SOC) tablosu
 * Termal     : 2 düğümlü (çekirdek-yüzey) toplu model, Bernardi tersinmez ısı, açık Euler
 * İşaret     : I > 0 deşarj, I < 0 şarj
 *
 * Parametreler bms/common/bms_config.h, simulink/bms_param_struct.m ve twin/battery_model.py ile
 * AYNI olmalı; biri değişirse burası da değişmeli.
 *
 * Hücre grupları (HIL adım 8): 19 seri grubun her biri ayrı SOC / RC durumu taşır. Gruplar arası
 * fark twin_model.c'deki sabit tabloyla verilir (kapasite, direnç, başlangıç SOC sapması; grup 7
 * "zayıf": -%5 kapasite, +%15 direnç, closed_loop.py'deki zayıf grubun hafifi). Pasif dengeleme:
 * maskede biti 1 olan grubun 12 hücresinden toplam i_bleed çekilir (direnç ısısı hücreye girmez).
 * Termal model ortalama hücre için tektir (gruplar arası sıcaklık farkı modellenmez).
 */
#ifndef TWIN_MODEL_H
#define TWIN_MODEL_H
#include <stdint.h>

#define TW_NG       19          /* seri grup sayısı */
#define TW_NS       19
#define TW_NP       12

typedef struct {
    float soc, v1, v2;          /* durum: SOC, RC1 ve RC2 gerilimleri (hücre) */
    float v;                    /* son adımdaki grup (hücre) uç gerilimi */
} tw_group_t;

typedef struct {
    tw_group_t g[TW_NG];
    float tc, ts;               /* ortalama hücre çekirdek / yüzey sıcaklığı [°C] */
    float t_amb;                /* ortam [°C], her adımdan önce dışarıdan yazılabilir */
    float a1, a2, dt;           /* RC ayrıklaştırma katsayıları (ZOH) */
    float v_pack, v_min, v_max; /* çıktı: paket gerilimi, en düşük / en yüksek grup gerilimi */
    float soc_avg;              /* çıktı: grup SOC ortalaması */
    float soc_min, soc_max;     /* çıktı: en düşük / en yüksek grup SOC'si */
} twin_t;

void  twin_init(twin_t *tw, float soc0, float t_amb, float dt);
void  twin_set_soc(twin_t *tw, float soc);       /* gruplara tablo sapmasıyla birlikte yazar */
/* bal_mask bit j = grup j+1'in dengeleme direnci açık, i_bleed = grup başına direnç akımı [A].
 * k_soc: zaman hızlandırma, direnç akımının SOC etkisi k_soc katı (1 = gerçek). Gerilim düşümü
 * gerçek akımla kalır: aksi hâlde dengelenen grup ECU'ya en düşük görünür ve anahtar titrer. */
void  twin_step(twin_t *tw, float i_pack, uint32_t bal_mask, float i_bleed, float k_soc);
float twin_ocv(float soc);      /* hücre OCV [V] */

#endif
