/* bms_bal.h -- pasif hücre dengeleme kararı (gerilim tabanlı), platformdan bağımsız.
 *
 * Girdi: 19 grup gerilimi (CAN 0x110-0x114), akım, ECU durumu, Tc_hat. Çıktı: 19 bitlik maske
 * (bit i = grup i+1'in deşarj direnci açık) ve bayraklar (bms_can.h CAN_BAL_F_*), CAN 0x210 ile.
 *
 * Kurallar (sabitler bms_config.h BAL_*):
 *   - Gerilimler BAL_FILT ile süzülür (ölçüm gürültüsünde anahtar titremesin).
 *   - İzin: sürüş yok (I <= BAL_I_DRIVE; şarj ve bekleme serbest), durum NORMAL/DERATE,
 *     Tc_hat <= BAL_T_MAX, en düşük grup >= BAL_V_MIN, tüm hücre çerçeveleri taze.
 *     İzin BAL_T_ON boyunca kesintisiz sürünce dengeleme etkin olur; izin bozulunca hemen kapanır.
 *   - Etkinken grup başına histerezis: V_i - V_min > BAL_DV_ON aç, < BAL_DV_OFF kapat. En düşük
 *     grup hiçbir zaman açılmaz. Pasif dengeleme yalnız SOC'yi hizalar; kapasite farkını düzeltemez.
 */
#ifndef BMS_BAL_H
#define BMS_BAL_H
#include <stdint.h>
#include "bms_config.h"

typedef struct {
    float    vf[BMS_NS];        /* süzülmüş grup gerilimleri */
    uint8_t  have_vf;
    uint16_t n_ok;              /* izin sayacı [adım] */
    uint32_t mask;              /* çıktı: açık dirençler */
    uint8_t  flags;             /* çıktı: CAN_BAL_F_* */
    float    v_min, v_max;      /* çıktı: süzülmüş en düşük / en yüksek grup [V] */
} bal_t;

void     bal_init(bal_t *b);
/* Her BMS adımında (BMS_TS). v_ok = 0: hücre verisi yok/eski -> maske 0, süzgeç sıfırlanır. */
uint32_t bal_step(bal_t *b, const float v[BMS_NS], uint8_t v_ok, float I, uint8_t state, float tc_hat);

#endif
