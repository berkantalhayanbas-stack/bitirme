/* mc_core.h -- kart üstü Monte Carlo: dijital ikiz + BMS (bms_step) kapalı çevrimi, hızlandırılmış.
 *
 * Her koşu (run) rastgele bir senaryodur: ortam sıcaklığı, başlangıç SOC'si, akım talebi ölçeği,
 * çevrim kaydırması, paket kapasite / direnç sapması, akım sensörü ofseti ve tek bir arıza
 * (yok / soğutma / dış ısı / NTC kopuk / büyük akım ofseti) zamanıyla. Rastgelelik xorshift32,
 * tohum = base_seed + run_id: aynı tohum STM32'de ve PC'de (tools/mc/mc_sil.c) aynı senaryoyu verir.
 * Kod platformdan bağımsız (printf, dinamik bellek yok); çıktı satırını kendisi biçimlendirir ki
 * iki platformun sonucu metin olarak karşılaştırılabilsin.
 *
 * Satırlar (gateway.py / dashboard protokolü, İngilizce):
 *   RS,<id>,<name>                                          koşu başladı
 *   RE,<id>,<PASS|FAIL>,<fault>,<react_ms>,<min_soc>,<max_tc>,<soc_err>,<final_state>,<tc_limp>,<tc_safe>,<why>
 * tc_limp / tc_safe: LIMP / SAFE anında gerçek çekirdek sıcaklığı (-99 = olmadı).
 * react_ms: arızaya beklenen tepkinin gecikmesi (-1 = tepki yok / arıza yok).
 */
#ifndef MC_CORE_H
#define MC_CORE_H
#include <stdint.h>
#include "bms_algo.h"

#define MC_STEPS   6000u        /* 600 s, Ts = 0,1 s */
enum { MC_F_NONE, MC_F_COOL, MC_F_HEAT, MC_F_NTC, MC_F_OFFSET, MC_F_N };

typedef struct {
    uint32_t id, seed;
    uint8_t  fault;
    float    t_amb, soc0, k_cur, kq, kr, bias, t_fault;
    uint32_t cyc_off;           /* çevrim başlangıç kaydırması [s] */
} mc_cfg_t;

typedef struct {
    uint8_t  pass, final_state;
    int32_t  react_ms;
    float    min_soc, max_tc, soc_err;   /* en düşük grup SOC'si, en yüksek gerçek çekirdek, en büyük |SOC hatası| */
    float    tc_limp, tc_safe;           /* LIMP / SAFE anında gerçek çekirdek sıcaklığı (-99 = yok) */
    int32_t  t_limp, t_safe, t_cool, t_sens;   /* ilk LIMP / SAFE / soğutma alarmı / sensör nedeni [ms], -1 yok */
    int32_t  t_anom;                     /* ilk termal anomali (neden 256) [ms], -1 yok */
    const char *why;                     /* FAIL nedeni (PASS'te "-") */
} mc_res_t;

/* isteğe bağlı iz kancası: boş değilse her 1 s'lik benzetim zamanında çağrılır (kart: D satırı) */
typedef void (*mc_trace_fn)(const mc_cfg_t *c, uint32_t t_ms, const bms_out_t *o);
extern mc_trace_fn mc_trace;

void        mc_config(uint32_t id, uint32_t base_seed, mc_cfg_t *c);
void        mc_run(const mc_cfg_t *c, mc_res_t *r);
const char *mc_fault_name(uint8_t f);
/* satırı buf'a yazar (sonunda "\r\n" yok, NUL ile biter), uzunluğu döner; buf >= 128 bayt */
uint32_t    mc_line_rs(const mc_cfg_t *c, char *buf);
uint32_t    mc_line_re(const mc_cfg_t *c, const mc_res_t *r, char *buf);

#endif
