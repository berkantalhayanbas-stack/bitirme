/* bsp.c -- NUCLEO-G431RB kart destek katmanı, kayıt seviyesinde (HAL yok).
 * Kaynak: RM0440 (STM32G4 Reference Manual), UM2505 (NUCLEO-G431RB kullanıcı kılavuzu).
 * Kayıt ve bit adları: stm32g431xx.h (ST, CMSIS).
 */
#include "stm32g4xx.h"
#include "bsp.h"

/* ------------------------------------------------------------------ FPU
 * Cortex-M4F'te kayan nokta birimi açılışta KAPALIDIR. Kod hard-float derlendiği için ilk float
 * komutu FPU kapalıyken HardFault verir. CPACR'da CP10 ve CP11'e tam erişim verilir. */
static void fpu_enable(void)
{
    SCB->CPACR |= (3UL << 20) | (3UL << 22);
    __DSB();
    __ISB();
}

/* ------------------------------------------------------------------ saat: 170 MHz
 * Açılışta işlemci HSI16 (dahili 16 MHz RC osilatör) ile çalışır. PLL:
 *   16 MHz / M(4) = 4 MHz  ->  x N(85) = 340 MHz (VCO)  ->  / R(2) = 170 MHz
 * 150 MHz üstü için RM0440 şartları:
 *   1) Voltaj regülatörü "Range 1 boost" modu (PWR_CR5.R1MODE = 0)
 *   2) Flash 4 bekleme durumu (0 WS ile işlemci flash'tan bu hızda okuyamaz)
 *   3) Geçişte AHB önce /2, en az 1 µs sonra /1 (ani akım sıçramasını önlemek için) */
static void clock_170mhz(void)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;                     /* PWR biriminin saatini aç */
    (void)RCC->APB1ENR1;                                      /* yazma tamamlansın */
    PWR->CR5 &= ~PWR_CR5_R1MODE;                              /* 1) boost modu */

    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | FLASH_ACR_LATENCY_4WS
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;      /* 2) */
    while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS) { }

    RCC->CR &= ~RCC_CR_PLLON;                                 /* PLL ayarı için kapalı olmalı */
    while (RCC->CR & RCC_CR_PLLRDY) { }
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI
                 | ((4U - 1U) << RCC_PLLCFGR_PLLM_Pos)        /* M = 4 (alana M-1 yazılır) */
                 | (85U << RCC_PLLCFGR_PLLN_Pos)              /* N = 85 */
                 | (0U << RCC_PLLCFGR_PLLR_Pos)               /* R = 2 (00 = /2) */
                 | RCC_PLLCFGR_PLLREN;                        /* PLLR çıkışını etkinleştir */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }                    /* PLL kilitlenene kadar bekle */

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE_Msk) | RCC_CFGR_HPRE_DIV2;   /* 3) önce /2 */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;        /* sistem saati = PLL */
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) { }
    for (volatile int i = 0; i < 200; i++) { }                           /* > 1 µs (85 MHz'de) */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE_Msk) | RCC_CFGR_HPRE_DIV1;   /* tam hız: 170 MHz */
}

/* ------------------------------------------------------------------ DWT döngü sayacı */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;           /* izleme birimini aç */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                      /* sayacı başlat */
}

uint32_t bsp_cycles(void) { return DWT->CYCCNT; }

void bsp_delay_ms(uint32_t ms)
{
    const uint32_t t0 = DWT->CYCCNT, n = ms * (BSP_CPU_HZ / 1000u);
    while ((uint32_t)(DWT->CYCCNT - t0) < n) { }              /* taşmaya dayanıklı fark */
}

/* ------------------------------------------------------------------ GPIO: LED (PA5) ve UART pinleri */
static void gpio_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;                      /* GPIOA saatini aç */
    (void)RCC->AHB2ENR;

    /* PA5: genel amaçlı çıkış (MODER = 01). Her pinin MODER'da 2 biti var: pin n -> bit 2n */
    GPIOA->MODER = (GPIOA->MODER & ~(3U << (5 * 2))) | (1U << (5 * 2));

    /* PA2, PA3: alternatif fonksiyon (MODER = 10), AF12 = LPUART1 (datasheet, AF tablosu).
     * AFR[0] pin 0-7'yi tutar, pin başına 4 bit. */
    GPIOA->MODER = (GPIOA->MODER & ~((3U << (2 * 2)) | (3U << (3 * 2))))
                 | (2U << (2 * 2)) | (2U << (3 * 2));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~((0xFU << (2 * 4)) | (0xFU << (3 * 4))))
                  | (12U << (2 * 4)) | (12U << (3 * 4));
}

void bsp_led_toggle(void) { GPIOA->ODR ^= (1U << 5); }

/* ------------------------------------------------------------------ LPUART1
 * Saat kaynağı PCLK1 (= 170 MHz, sıfırlama sonrası varsayılan). LPUART'ta baud bölücü
 * BRR = 256 * f_saat / baud  (normal USART'tan farklı olarak 256 ile çarpılır). */
static void lpuart1_init(void)
{
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    (void)RCC->APB1ENR2;
    RCC->CCIPR &= ~RCC_CCIPR_LPUART1SEL;                      /* 00 = PCLK1 */
    LPUART1->CR1 = 0;                                         /* ayar için kapalı (UE = 0) */
    LPUART1->BRR = (uint32_t)(((uint64_t)256u * BSP_CPU_HZ + BSP_BAUD / 2u) / BSP_BAUD);
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  /* 8N1, gönder + al + aç */
    NVIC_EnableIRQ(LPUART1_IRQn);                             /* yalnız TXEIE açılınca (async) tetiklenir */
}

/* Kesmeli gönderim (app_ecu): halka tampon + TXE kesmesi. Bloklayan yazma, CAN Rx FIFO'su (3
 * çerçeve) boşaltılamadan ~30 ms beklediği için hücre çerçevelerini kaybettiriyordu (adım 8). */
#define TXQ_N 2048u
static volatile uint8_t  txq[TXQ_N];
static volatile uint32_t txq_h, txq_t, txq_drop;

uint32_t bsp_uart_write_async(const uint8_t *buf, uint32_t n)
{
    uint32_t k = 0;
    for (; k < n; k++) {
        const uint32_t next = (txq_h + 1u) % TXQ_N;
        if (next == txq_t) { txq_drop += n - k; break; }       /* tampon dolu: kalan atılır */
        txq[txq_h] = buf[k];
        txq_h = next;
    }
    LPUART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;                  /* kesme boşalınca kendini kapatır */
    return k;
}
uint32_t bsp_uart_tx_drops(void) { return txq_drop; }

/* Kesmeli alım: 256 B halka tampon (komut satırları kısa; dolarsa yeni bayt atılır) */
#define RXQ_N 256u
static volatile uint8_t  rxq[RXQ_N];
static volatile uint32_t rxq_h, rxq_t;

void bsp_uart_rx_enable(void)
{
    rxq_h = rxq_t = 0;
    LPUART1->ICR = USART_ICR_ORECF;
    LPUART1->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
}

int bsp_uart_getc(uint8_t *c)
{
    if (rxq_t == rxq_h) return 0;
    *c = rxq[rxq_t];
    rxq_t = (rxq_t + 1u) % RXQ_N;
    return 1;
}

void LPUART_IRQHandler(void)                                  /* vektör adı başlangıç dosyasındaki gibi */
{
    if (LPUART1->ISR & USART_ISR_ORE) LPUART1->ICR = USART_ICR_ORECF;     /* taşma: bayrağı sil */
    if ((LPUART1->CR1 & USART_CR1_RXNEIE_RXFNEIE) && (LPUART1->ISR & USART_ISR_RXNE_RXFNE)) {
        const uint8_t b = (uint8_t)LPUART1->RDR;
        const uint32_t next = (rxq_h + 1u) % RXQ_N;
        if (next != rxq_t) { rxq[rxq_h] = b;  rxq_h = next; }
    }
    if ((LPUART1->CR1 & USART_CR1_TXEIE_TXFNFIE) && (LPUART1->ISR & USART_ISR_TXE_TXFNF)) {
        if (txq_t != txq_h) { LPUART1->TDR = txq[txq_t];  txq_t = (txq_t + 1u) % TXQ_N; }
        else LPUART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    }
}

void bsp_uart_write(const uint8_t *buf, uint32_t n)
{
    while (txq_t != txq_h) { }                                /* sıra bozulmasın: kuyruk boşalsın */
    for (uint32_t i = 0; i < n; i++) {
        while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF)) { }    /* veri kaydı boşalsın */
        LPUART1->TDR = buf[i];
    }
    while (!(LPUART1->ISR & USART_ISR_TC)) { }                /* son bayt da çıksın */
}

void bsp_reset(void)
{
    NVIC_SystemReset();                                       /* SCB->AIRCR SYSRESETREQ */
}

void bsp_uart_read(uint8_t *buf, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        while (!(LPUART1->ISR & USART_ISR_RXNE_RXFNE)) {
            if (LPUART1->ISR & USART_ISR_ORE) LPUART1->ICR = USART_ICR_ORECF;  /* taşma: bayrağı sil */
        }
        buf[i] = (uint8_t)LPUART1->RDR;
    }
}

/* Bazı başlangıç (startup) dosyaları main'den önce SystemInit() çağırır, boş projeninki çağırmaz.
 * "weak" tanım: başka yerde SystemInit varsa o kullanılır, yoksa bu boş fonksiyon. Saat kurulumu
 * zaten bsp_init içinde yapılıyor. */
__attribute__((weak)) void SystemInit(void) { }

/* ------------------------------------------------------------------ hepsi */
void bsp_init(void)
{
    fpu_enable();
    clock_170mhz();
    dwt_init();
    gpio_init();
    lpuart1_init();
}
