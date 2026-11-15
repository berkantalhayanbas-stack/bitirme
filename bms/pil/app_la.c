/* app_la.c -- Adım 5: STM32'nin kendi lojik analizörü, CAN RX pini PA11 (APP_LA).
 *
 * Örnekleme: TIM6 güncellemesi (170 MHz / 40 = 4,25 MHz; 4 MHz'e tam bölünmez) -> DMAMUX istek 8
 *   (TIM6_UP) -> DMA1 kanal 1: GPIOA->IDR'nin yüksek baytı (PA8..PA15; PA11 = bit 3) -> RAM,
 *   16 384 örnek = 3,855 ms. Bayt okuma: 16 KB (yarım kelime 32 KB olurdu, RAM 32 KB).
 * Tetik: hat en az LA_IDLE_US boyunca çekinik (1) kaldıktan sonraki ilk düşen kenar (SOF).
 *   Yazılım yoklamasıyla: kenardan TIM6'nın başlamasına ~0,1 µs, ilk örnek bir periyot (0,235 µs)
 *   sonra; SOF'un (2 µs) başı biraz kesilir, çözümü etkilemez.
 * CAN: FDCAN1 normal modda açık (hatta tek alıcı bu düğümse ACK'i o verir; ACK biti RX'te görünür).
 *
 * UART protokolü (115200, ikili):
 *   PC -> kart: 'C'  (yakala)
 *   kart -> PC: "LA" | rate_hz u32 | n_samples u16 | bit u8 (11) | status u8 (0 tamam, 1 tetik yok)
 *               | veri: n/8 bayt, örnekler bitlere paketli, MSB önce | crc8_j1850(rate..veri)
 * Çözüm ve çizim PC'de: tools/la_view.py (bit doldurma çözülür, alanlar işaretlenir, bit süresi).
 */
#include <string.h>
#include "stm32g4xx.h"
#include "app_la.h"
#include "bsp.h"
#include "bsp_can.h"
#include "bms_can.h"           /* crc8_j1850 */

#define LA_N          16384u
#define LA_DIV        40u                       /* 170 MHz / 40 = 4,25 MHz */
#define LA_RATE_HZ    (BSP_CPU_HZ / LA_DIV)
#define LA_IDLE_US    30u                       /* > 11 bit (22 µs): çerçeve arası boşluk */
#define LA_TIMEOUT_MS 2000u
#define DMAREQ_TIM6_UP 8u                       /* RM0440 DMAMUX tablosu / HAL DMA_REQUEST_TIM6_UP */

static uint8_t buf[LA_N];
static uint8_t pack[LA_N / 8];

static void la_init(void)
{
    RCC->AHB1ENR  |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    (void)RCC->APB1ENR1;
    TIM6->CR1 = 0;
    TIM6->PSC = 0;
    TIM6->ARR = LA_DIV - 1u;
    TIM6->EGR = TIM_EGR_UG;                     /* PSC/ARR yüklensin */
    TIM6->SR  = 0;
    TIM6->DIER = TIM_DIER_UDE;                  /* güncelleme -> DMA isteği */
    DMAMUX1_Channel0->CCR = DMAREQ_TIM6_UP;     /* DMA1 kanal 1 <- DMAMUX kanal 0 */
}

/* 1: yakalandı, 0: tetik gelmedi */
static int la_capture(void)
{
    const uint32_t idle_cyc = LA_IDLE_US * (BSP_CPU_HZ / 1000000u);
    const uint32_t t_end = bsp_cycles() + LA_TIMEOUT_MS * (BSP_CPU_HZ / 1000u);

    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMA1_Channel1->CPAR  = (uint32_t)&GPIOA->IDR + 1u;   /* yüksek bayt: PA8..PA15 */
    DMA1_Channel1->CMAR  = (uint32_t)buf;
    DMA1_Channel1->CNDTR = LA_N;
    DMA1_Channel1->CCR   = DMA_CCR_PL_1 | DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_EN;  /* 8 bit -> 8 bit */
    TIM6->CNT = 0;

    /* tetik: >= LA_IDLE_US çekinik, sonra düşen kenar */
    uint32_t t_rec = bsp_cycles();
    for (;;) {
        const uint32_t now = bsp_cycles();
        if ((int32_t)(now - t_end) >= 0) { DMA1_Channel1->CCR = 0; return 0; }
        if (GPIOA->IDR & (1u << 11)) continue;
        if (now - t_rec >= idle_cyc) break;     /* boşluktan sonra ilk dominant: SOF */
        while (!(GPIOA->IDR & (1u << 11))) { }  /* çerçeve içi dominant: çekinik olmasını bekle */
        t_rec = bsp_cycles();
    }
    TIM6->CR1 = TIM_CR1_CEN;
    while (!(DMA1->ISR & DMA_ISR_TCIF1)) { }
    TIM6->CR1 = 0;
    DMA1_Channel1->CCR = 0;
    return 1;
}

void la_run(void)
{
    for (int k = 0; k < 6; k++) { bsp_led_toggle(); bsp_delay_ms(150); }
    static const char hello[] = "\r\n=== STM32 logic analyzer: PA11 (CAN RX), 4.25 MHz, 16384 samples; send 'C' ===\r\n";
    bsp_uart_write((const uint8_t *)hello, sizeof(hello) - 1);
    bsp_can_init_normal();                      /* ACK verebilmek için hatta katıl */
    la_init();

    for (;;) {
        uint8_t c;
        bsp_uart_read(&c, 1);
        if (c != 'C') continue;
        const int ok = la_capture();
        { uint16_t id; uint8_t d[8], len; while (bsp_can_recv(&id, d, &len)) { } }   /* FIFO'yu boşalt */

        memset(pack, 0, sizeof(pack));
        if (ok)
            for (uint32_t i = 0; i < LA_N; i++)
                if (buf[i] & (1u << 3)) pack[i >> 3] |= (uint8_t)(0x80u >> (i & 7u));

        uint8_t h[10] = { 'L', 'A' };
        const uint32_t rate = LA_RATE_HZ;
        const uint16_t n = ok ? (uint16_t)LA_N : 0u;
        memcpy(&h[2], &rate, 4);
        memcpy(&h[6], &n, 2);
        h[8] = 11u;
        h[9] = ok ? 0u : 1u;
        /* CRC kapsamı: 'LA'dan sonraki başlık (8 bayt) + veri */
        static uint8_t crc_in[8 + LA_N / 8];
        memcpy(crc_in, &h[2], 8);
        memcpy(&crc_in[8], pack, n / 8u);
        const uint8_t crc = crc8_j1850(crc_in, (int)(8u + n / 8u));
        bsp_uart_write(h, 10);
        if (n) bsp_uart_write(pack, n / 8u);
        bsp_uart_write(&crc, 1);
        bsp_led_toggle();
    }
}
