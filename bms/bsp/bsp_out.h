/* bsp_out.h -- araç çıkışları (adım 4): far PWM'leri ve kontaktör rölesi, ULN2003AN üzerinden.
 *
 *   Kısa far  PA8 / TIM1_CH1 (AF6) -> ULN2003 IN1   (Nucleo D7)
 *   Uzun far  PA9 / TIM1_CH2 (AF6) -> ULN2003 IN2   (Nucleo D8)  aynı timer, aynı 1 kHz
 *   Kontaktör PB5 GPIO             -> ULN2003 IN3   (Nucleo D4)  1 = röle çekili (kontaktör kapalı)
 * TIM1: 170 MHz / 170 = 1 MHz sayıcı, ARR 999 -> 1 kHz, duty çözünürlüğü %0,1. ULN2003 girişi
 * 2,7 kΩ seri dirençli; 3,3 V lojikle sürülür; çıkış açık kollektör (yükü toprağa çeker),
 * COM ucu röle/LED beslemesine bağlanır (dahili serbest dönüş diyotu).
 * Açılış: çıkışlar 0 (farlar sönük, kontaktör açık) — güvenli varsayılan.
 */
#ifndef BSP_OUT_H
#define BSP_OUT_H
#include <stdint.h>

#define BSP_OUT_PWM_HZ 1000u

void bsp_out_init(void);
void bsp_out_set(float duty_low, float duty_high, uint8_t contactor);   /* duty 0..1 */

/* Pin seviyesinde doğrulama: PA8/PA9'u window_us boyunca yazılımla örnekler (IDR, AF modunda da
 * pin seviyesini verir); frekans (yükselen kenar sayısı) ve doluluk; PB5'in çıkış seviyesi. */
typedef struct { uint32_t f_low_hz, f_high_hz; float d_low, d_high; uint8_t contactor; } bsp_out_meas_t;
void bsp_out_measure(uint32_t window_us, bsp_out_meas_t *m);

#endif
