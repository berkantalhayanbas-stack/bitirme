/* bsp_can.c -- FDCAN1, kayıt seviyesinde. Kaynak: RM0440 bölüm 44 (FDCAN), STM32G431 datasheet
 * (alternatif fonksiyon tablosu: PA11 FDCAN1_RX AF9, PA12 FDCAN1_TX AF9).
 *
 * STM32G4'te mesaj RAM'i (SRAMCAN) yerleşimi SABİTTİR, yazılımla ayarlanmaz. FDCAN1 için
 * SRAMCAN_BASE'ten itibaren (bayt ofseti):
 *   0x000  11-bit filtreler   28 x 4 bayt
 *   0x070  29-bit filtreler    8 x 8 bayt
 *   0x0B0  Rx FIFO 0           3 x 72 bayt
 *   0x188  Rx FIFO 1           3 x 72 bayt
 *   0x260  Tx olay FIFO'su     3 x 8 bayt
 *   0x278  Tx tamponları       3 x 72 bayt   -> toplam 0x350 bayt
 * Her Rx/Tx elemanı 18 kelimedir (2 başlık + 64 bayt veri), klasik CAN'da da.
 */
#include "stm32g4xx.h"
#include "bsp.h"
#include "bsp_can.h"

#define RAM_FLS    (SRAMCAN_BASE + 0x000u)
#define RAM_RXF0   (SRAMCAN_BASE + 0x0B0u)
#define RAM_TXB    (SRAMCAN_BASE + 0x278u)
#define RAM_SIZE   0x350u
#define ELEM_SIZE  72u

/* Bit zamanlaması, 500 kbit/s:  170 MHz / 17 = 10 MHz -> 20 tq/bit
 *   1 (sync) + 15 (TSEG1) + 4 (TSEG2) = 20 tq, örnekleme noktası %80, SJW = 4 tq.
 * NBTP alanlarına (değer - 1) yazılır. */
#define NBRP    17u
#define NTSEG1  15u
#define NTSEG2   4u
#define NSJW     4u
#if (BSP_CPU_HZ / (NBRP * (1u + NTSEG1 + NTSEG2))) != BSP_CAN_BITRATE
#error "FDCAN bit zamanlamasi BSP_CAN_BITRATE ile uyusmuyor"
#endif

/* Standart kimlik filtre elemanı (RM0440 44.3.11, "Standard message ID filter element"):
 *   SFT  [31:30]  01 = ikili kimlik: SFID1 veya SFID2'ye eşit olan kabul
 *   SFEC [29:27]  001 = eşleşeni Rx FIFO 0'a koy
 *   SFID1[26:16], SFID2[10:0] */
#define SFE_DUAL_FIFO0(id1, id2)  ((1u << 30) | (1u << 27) | (((uint32_t)(id1) & 0x7FFu) << 16) \
                                   | ((uint32_t)(id2) & 0x7FFu))
/* SFT = 00: aralık filtresi, SFID1 <= kimlik <= SFID2 kabul */
#define SFE_RANGE_FIFO0(lo, hi)   ((0u << 30) | (1u << 27) | (((uint32_t)(lo) & 0x7FFu) << 16) \
                                   | ((uint32_t)(hi) & 0x7FFu))

/* RXGFC: ANFS/ANFE = 10 eşleşmeyen std/ext çerçeveyi reddet, RRFS/RRFE = 1 uzak çerçeveleri reddet */
#define RXGFC_REJECT_ALL  ((2u << FDCAN_RXGFC_ANFS_Pos) | (2u << FDCAN_RXGFC_ANFE_Pos) \
                           | (1u << FDCAN_RXGFC_RRFS_Pos) | (1u << FDCAN_RXGFC_RRFE_Pos))

enum { MODE_LOOPBACK, MODE_LOOPBACK_FILTERED, MODE_NORMAL };

/* birikimli hata sayaçları */
static uint32_t s_cel, s_busoff, s_recover, s_rx_lost;
static uint8_t  s_lec;               /* son anlamlı LEC (PSR okununca donanımda 7'ye döner) */
/* bus-off kurtarma durumu */
static uint8_t  s_bo_wait;           /* bus-off görüldü, bekleme süresi işliyor */
static uint32_t s_bo_t0;             /* bus-off'un görüldüğü an [ms] */
static uint32_t s_rec_t;             /* son kurtarma anı [ms] */
static uint32_t s_streak;            /* ardışık kurtarma sayısı */

static uint8_t dlc_len(uint32_t dlc) { return (uint8_t)(dlc > 8u ? 8u : dlc); }

/* ------------------------------------------------------------------ pinler: PA11 RX, PA12 TX, AF9 */
static void pins_init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC->AHB2ENR;
    /* MODER = 10 (alternatif fonksiyon), pin n -> bit 2n */
    GPIOA->MODER = (GPIOA->MODER & ~((3u << (11 * 2)) | (3u << (12 * 2))))
                 | (2u << (11 * 2)) | (2u << (12 * 2));
    /* TX için yüksek hız (kenar dikliği). İki pinde de pull-up/pull-down YOK (PUPDR = 00):
     * TJA1050 5 V ile beslenir ve RXD'yi 5 V'a kadar sürer. PA11/PA12 FT_u (5 V toleranslı) ama
     * veri sayfası (DS12589) VDD + 0,3 V üstünde iç dirençlerin kapalı olmasını şart koşar.
     * Transceiver RXD'yi her zaman sürdüğü için pull-up'a gerek de yok. */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3u << (12 * 2))) | (2u << (12 * 2));
    GPIOA->PUPDR &= ~((3u << (11 * 2)) | (3u << (12 * 2)));
    /* AFR[1] pin 8..15'i tutar: pin n -> bit 4(n-8) */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((0xFu << ((11 - 8) * 4)) | (0xFu << ((12 - 8) * 4))))
                  | (9u << ((11 - 8) * 4)) | (9u << ((12 - 8) * 4));
}

/* ------------------------------------------------------------------ ortak yapılandırma */
static void can_init(int mode)
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_FDCANSEL) | RCC_CCIPR_FDCANSEL_1;   /* 10 = PCLK1 */

    if (mode == MODE_NORMAL) pins_init();

    /* Yapılandırma için başlatma moduna gir: INIT=1, sonra CCE=1 (korumalı kayıtlara yazma izni) */
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    while (!(FDCAN1->CCCR & FDCAN_CCCR_INIT)) { }
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    /* Klasik CAN (FDOE=0, BRSE=0), otomatik yeniden gönderme açık (DAR=0), TEST/MON önce kapalı.
     * CCCR.TEST = 0 olunca TEST kaydı (LBCK dahil) sıfırlanır. */
    FDCAN1->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR
                      | FDCAN_CCCR_TEST | FDCAN_CCCR_MON);
    if (mode != MODE_NORMAL) {                                /* iç geri döngü = TEST + LBCK + MON */
        FDCAN1->CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
        FDCAN1->TEST |= FDCAN_TEST_LBCK;
    }

    FDCAN1->NBTP = ((NSJW - 1u) << FDCAN_NBTP_NSJW_Pos) | ((NBRP - 1u) << FDCAN_NBTP_NBRP_Pos)
                 | ((NTSEG1 - 1u) << FDCAN_NBTP_NTSEG1_Pos) | ((NTSEG2 - 1u) << FDCAN_NBTP_NTSEG2_Pos);
    FDCAN1->TXBC = 0;                                         /* Tx FIFO modu */

    /* Mesaj RAM'i sıfırlanmış durumda değil; filtre alanı dahil temizle */
    for (uint32_t a = SRAMCAN_BASE; a < SRAMCAN_BASE + RAM_SIZE; a += 4u) *(volatile uint32_t *)a = 0;

    if (mode == MODE_LOOPBACK) {
        FDCAN1->RXGFC = 0;                    /* filtre yok, eşleşmeyen std/ext -> Rx FIFO 0 */
    } else {
        static const uint16_t ids[] = BSP_CAN_FILTER_IDS;
        volatile uint32_t *fls = (volatile uint32_t *)RAM_FLS;
        fls[0] = SFE_DUAL_FIFO0(ids[0], ids[1]);
        fls[1] = SFE_DUAL_FIFO0(ids[2], ids[2]);
        fls[2] = SFE_RANGE_FIFO0(BSP_CAN_FILTER_RANGE_LO, BSP_CAN_FILTER_RANGE_HI);
        FDCAN1->RXGFC = RXGFC_REJECT_ALL | (3u << FDCAN_RXGFC_LSS_Pos);   /* 3 std filtre elemanı */
    }

    FDCAN1->IR = 0xFFFFFFFFu;                                 /* eski kesme bayraklarını temizle */
    (void)FDCAN1->ECR;                                        /* CEL'i sıfırla (okuyunca sıfırlanır) */
    s_cel = s_busoff = s_recover = s_rx_lost = 0;
    s_lec = 0;
    s_bo_wait = 0;  s_streak = 0;  s_rec_t = 0;

    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;                         /* çalışmaya başla (CCE kendiliğinden 0) */
    while (FDCAN1->CCCR & FDCAN_CCCR_INIT) { }
}

void bsp_can_init_loopback(void)          { can_init(MODE_LOOPBACK); }
void bsp_can_init_loopback_filtered(void) { can_init(MODE_LOOPBACK_FILTERED); }
void bsp_can_init_normal(void)            { can_init(MODE_NORMAL); }

/* ------------------------------------------------------------------ gönderme / alma */
int bsp_can_send(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (FDCAN1->CCCR & FDCAN_CCCR_INIT) return 0;             /* bus-off ya da yapılandırmada */
    if (FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) return 0;
    if (len > 8u) len = 8u;
    const uint32_t idx = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    volatile uint32_t *e = (volatile uint32_t *)(RAM_TXB + idx * ELEM_SIZE);

    uint32_t w[2] = { 0, 0 };
    for (uint32_t i = 0; i < len; i++) w[i / 4u] |= (uint32_t)data[i] << (8u * (i % 4u));
    e[0] = ((uint32_t)id & 0x7FFu) << 18;                     /* T0: std ID bit 28..18, XTD=RTR=0 */
    e[1] = (uint32_t)len << 16;                               /* T1: DLC, FDF=BRS=0, olay kaydı yok */
    e[2] = w[0];
    e[3] = w[1];
    FDCAN1->TXBAR = 1u << idx;                                /* gönderim isteği */
    return 1;
}

int bsp_can_recv(uint16_t *id, uint8_t *data, uint8_t *len)
{
    if ((FDCAN1->RXF0S & FDCAN_RXF0S_F0FL) == 0) return 0;
    const uint32_t idx = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    const volatile uint32_t *e = (const volatile uint32_t *)(RAM_RXF0 + idx * ELEM_SIZE);

    const uint32_t r0 = e[0], r1 = e[1], w[2] = { e[2], e[3] };
    *id  = (uint16_t)((r0 >> 18) & 0x7FFu);
    *len = dlc_len((r1 >> 16) & 0xFu);
    for (uint32_t i = 0; i < *len; i++) data[i] = (uint8_t)(w[i / 4u] >> (8u * (i % 4u)));
    FDCAN1->RXF0A = idx;                                      /* elemanı serbest bırak */
    return 1;
}

/* ------------------------------------------------------------------ hata yönetimi */
static void collect(void)
{   /* okununca sıfırlanan alanları biriktir */
    s_cel += (FDCAN1->ECR & FDCAN_ECR_CEL_Msk) >> FDCAN_ECR_CEL_Pos;
    const uint8_t lec = (uint8_t)(FDCAN1->PSR & FDCAN_PSR_LEC);
    if (lec != 7u) s_lec = lec;                               /* 7 = son okumadan beri değişiklik yok */
    if (FDCAN1->IR & FDCAN_IR_RF0L) { FDCAN1->IR = FDCAN_IR_RF0L; s_rx_lost++; }
}

int bsp_can_service(uint32_t now_ms)
{
    collect();
    if (FDCAN1->IR & FDCAN_IR_BO) {                           /* BO durum değişimi bayrağı */
        FDCAN1->IR = FDCAN_IR_BO;
    }
    /* Bus-off'ta donanım INIT=1 yapar. INIT temizlenince denetleyici 129 kez 11 resesif bit
     * görmeyi bekler, sonra TEC/REC sıfırlanmış olarak hatta döner (RM0440 44.3.2). */
    if (!((FDCAN1->PSR & FDCAN_PSR_BO) && (FDCAN1->CCCR & FDCAN_CCCR_INIT))) return 0;

    if (!s_bo_wait) {                                         /* yeni bus-off olayı */
        s_bo_wait = 1;
        s_bo_t0 = now_ms;
        s_busoff++;
        if (s_streak && (now_ms - s_rec_t) >= BSP_CAN_BO_SLOW_MS) s_streak = 0;   /* uzun süre sağlıklıydı */
    }
    const uint32_t hold = (s_streak < BSP_CAN_BO_FAST_N) ? BSP_CAN_BO_FAST_MS : BSP_CAN_BO_SLOW_MS;
    if ((now_ms - s_bo_t0) < hold) return 0;

    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    s_bo_wait = 0;
    s_rec_t = now_ms;
    s_streak++;
    s_recover++;
    return 1;
}

void bsp_can_errors(bsp_can_err_t *e)
{
    collect();
    const uint32_t ecr = FDCAN1->ECR, psr = FDCAN1->PSR;
    s_cel += (ecr & FDCAN_ECR_CEL_Msk) >> FDCAN_ECR_CEL_Pos;  /* bu okumadaki CEL de kaybolmasın */
    e->tec  = (uint8_t)((ecr & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos);
    e->rec  = (uint8_t)((ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos);
    e->rp   = (uint8_t)((ecr & FDCAN_ECR_RP_Msk) != 0);
    e->ew   = (uint8_t)((psr & FDCAN_PSR_EW) != 0);
    e->ep   = (uint8_t)((psr & FDCAN_PSR_EP) != 0);
    e->bo   = (uint8_t)((psr & FDCAN_PSR_BO) != 0);
    if ((psr & FDCAN_PSR_LEC) != 7u) s_lec = (uint8_t)(psr & FDCAN_PSR_LEC);
    e->lec  = s_lec;
    e->init = (uint8_t)((FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0);
    e->cel = s_cel;  e->n_busoff = s_busoff;  e->n_recover = s_recover;  e->n_rx_lost = s_rx_lost;
}

void bsp_can_clear_stats(void)
{
    collect();                                   /* bekleyen olaylar da sayılıp silinsin */
    (void)FDCAN1->ECR;                           /* CEL okununca sıfırlanır */
    s_cel = 0;  s_busoff = 0;  s_recover = 0;  s_rx_lost = 0;
}
