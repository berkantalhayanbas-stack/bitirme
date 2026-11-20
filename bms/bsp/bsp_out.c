/* bsp_out.c -- far PWM (TIM1 CH1/CH2, PA8/PA9) ve kontaktör (PB5), bkz. bsp_out.h. */
#include "stm32g4xx.h"
#include "bsp.h"
#include "bsp_out.h"

#define OUT_ARR (1000000u / BSP_OUT_PWM_HZ - 1u)     /* 1 MHz sayıcıda 999 */

void bsp_out_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

#if BSP_OUT_K_OPEN_DRAIN
    /* PB5: önce açık-akaçlı + ODR 1 (yüksek empedans), sonra çıkış -> açılışta röle çekmez */
    GPIOB->OTYPER |= 1u << 5;
    GPIOB->BSRR = 1u << 5;
#else
    /* PB5: önce 0, sonra çıkış (push-pull) -> açılışta röle çekmez */
    GPIOB->OTYPER &= ~(1u << 5);
    GPIOB->BSRR = 1u << (5 + 16);
#endif
    GPIOB->MODER = (GPIOB->MODER & ~(3u << (5 * 2))) | (1u << (5 * 2));

    /* TIM1: PWM mod 1, önyüklemeli; CCR = 0 -> çıkışlar düşük */
    TIM1->CR1 = 0;
    TIM1->PSC = BSP_CPU_HZ / 1000000u - 1u;      /* 1 MHz */
    TIM1->ARR = OUT_ARR;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;  /* aktif yüksek */
    TIM1->BDTR = TIM_BDTR_MOE;                   /* gelişmiş timer: ana çıkış izni */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    /* PA8, PA9: AF6 (TIM1_CH1 / CH2) */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFu << ((8 - 8) * 4)) | (0xFu << ((9 - 8) * 4))))
                  | (6u << ((8 - 8) * 4)) | (6u << ((9 - 8) * 4));
    GPIOA->MODER = (GPIOA->MODER & ~((3u << (8 * 2)) | (3u << (9 * 2)))) | (2u << (8 * 2)) | (2u << (9 * 2));
}

static uint32_t ccr(float d)
{
    if (!(d > 0.0f)) return 0u;
    if (d >= 1.0f) return OUT_ARR + 1u;              /* %100: CCR > ARR -> sürekli yüksek */
    return (uint32_t)(d * (float)(OUT_ARR + 1u) + 0.5f);
}

void bsp_out_set(float duty_low, float duty_high, uint8_t contactor)
{    TIM1->CCR1 = ccr(duty_low);
    TIM1->CCR2 = ccr(duty_high);
#if BSP_OUT_K_OPEN_DRAIN
    GPIOB->BSRR = contactor ? (1u << (5 + 16)) : (1u << 5);     /* çekili = 0 V, açık = bırak */
#else
    GPIOB->BSRR = contactor ? (1u << 5) : (1u << (5 + 16));
#endif
}

void bsp_out_measure(uint32_t window_us, bsp_out_meas_t *m, void (*poll)(void))
{
    const uint32_t t0 = bsp_cycles(), len = window_us * (BSP_CPU_HZ / 1000000u);
    const uint32_t poll_cyc = 200u * (BSP_CPU_HZ / 1000000u);
    uint32_t t_poll = t0;
    uint32_t n = 0, hi8 = 0, hi9 = 0, r8 = 0, r9 = 0, prev = GPIOA->IDR;
    uint32_t a8 = 0, b8 = 0, a9 = 0, b9 = 0;     /* ilk / son yükselen kenarın DWT zamanı */
    while (bsp_cycles() - t0 < len) {
        const uint32_t v = GPIOA->IDR, now = bsp_cycles(), rise = v & ~prev;
        n++;
        hi8 += (v >> 8) & 1u;
        hi9 += (v >> 9) & 1u;
        if (rise & (1u << 8)) { if (!r8++) a8 = now; b8 = now; }
        if (rise & (1u << 9)) { if (!r9++) a9 = now; b9 = now; }
        prev = v;
        if (poll && now - t_poll >= poll_cyc) { t_poll = now;  poll(); }   /* kısa: örnekleme birkaç µs kayar */
    }
    m->d_low  = (float)hi8 / (float)n;
    m->d_high = (float)hi9 / (float)n;
    /* frekans = (kenar sayısı - 1) periyot / ilk..son kenar arası süre; %0 / %100'de 0 Hz */
    m->f_low_hz  = r8 > 1u ? (uint32_t)((uint64_t)(r8 - 1u) * BSP_CPU_HZ / (b8 - a8)) : 0u;
    m->f_high_hz = r9 > 1u ? (uint32_t)((uint64_t)(r9 - 1u) * BSP_CPU_HZ / (b9 - a9)) : 0u;
    m->contactor = (uint8_t)(((GPIOB->ODR >> 5) & 1u) ^ (BSP_OUT_K_OPEN_DRAIN ? 1u : 0u));
    m->k_pin = (uint8_t)((GPIOB->IDR >> 5) & 1u);
}
