/* bsp_out.h -- araç çıkışları (adım 4): far PWM'leri ve kontaktör rölesi, ULN2003AN üzerinden.
 *
 *   Kısa far  PA8 / TIM1_CH1 (AF6) -> ULN2003 IN1   (Nucleo D7)
 *   Uzun far  PA9 / TIM1_CH2 (AF6) -> ULN2003 IN2   (Nucleo D8)  aynı timer, aynı 1 kHz
 *   Kontaktör PB5 GPIO             -> röle modülü IN (Nucleo D4)  1 = röle çekili (kontaktör kapalı)
 * TIM1: 170 MHz / 170 = 1 MHz sayıcı, ARR 999 -> 1 kHz, duty çözünürlüğü %0,1. ULN2003 girişi
 * 2,7 kΩ seri dirençli; 3,3 V lojikle sürülür; çıkış açık kollektör (yükü toprağa çeker),
 * COM ucu röle/LED beslemesine bağlanır (dahili serbest dönüş diyotu).
 * Açılış: çıkışlar 0 (farlar sönük, kontaktör açık) — güvenli varsayılan.
 *
 * Kontaktör sürüşü (BSP_OUT_K_OPEN_DRAIN):
 *   1 (varsayılan): 5 V beslemeli, düşükte tetiklenen optokuplörlü röle modülü. IN'e 3,3 V
 *     verilince de çekiyor (5 V - 3,3 V optokuplör LED'ini sürmeye yetiyor; tezgâhta denendi,
 *     2026-11-16), yalnız IN boşta kalınca bırakıyor. Bu yüzden PB5 açık-akaçlı: çekili = 0 V,
 *     açık = yüksek empedans (modülün kendi çekme direnci IN'i 5 V'a çeker; PB5 FT, 5 V toleranslı).
 *     Reset / açılışta pin analog (yüksek empedans) -> röle bırakır: donanımda da güvenli varsayılan.
 *   0: itme-çekme, yüksekte aktif (ULN2003 IN3 gibi akım çeken sürücüler için).
 */
#ifndef BSP_OUT_H
#define BSP_OUT_H
#include <stdint.h>

#define BSP_OUT_PWM_HZ 1000u
#ifndef BSP_OUT_K_OPEN_DRAIN
#define BSP_OUT_K_OPEN_DRAIN 1
#endif

void bsp_out_init(void);
void bsp_out_set(float duty_low, float duty_high, uint8_t contactor);   /* duty 0..1 */

/* Pin seviyesinde doğrulama: PA8/PA9'u window_us boyunca yazılımla örnekler (IDR, AF modunda da
 * pin seviyesini verir); frekans (yükselen kenar sayısı) ve doluluk; kontaktör komutu (ODR'den,
 * 1 = çekili) ve PB5'in gerçek pin seviyesi (IDR; açık-akaçlıda bırakınca modül 1'e çeker). */
typedef struct { uint32_t f_low_hz, f_high_hz; float d_low, d_high; uint8_t contactor, k_pin; } bsp_out_meas_t;
/* poll (NULL olabilir): ölçüm sürerken ~200 µs'de bir çağrılır. 20 ms boyunca CAN Rx FIFO'su (3
 * çerçeve) boşaltılmazsa taşıyordu (saniyede 1 kayıp hücre çerçevesi, lost#). */
void bsp_out_measure(uint32_t window_us, bsp_out_meas_t *m, void (*poll)(void));

#endif
