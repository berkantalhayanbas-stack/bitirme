/* bsp_can.h -- FDCAN1 sürücüsü, kayıt seviyesinde (HAL yok), klasik CAN 500 kbit/s.
 *
 *   Saat   : FDCAN çekirdek saati = PCLK1 (170 MHz), 20 tq/bit, örnekleme noktası %80
 *   Alım   : Rx FIFO 0, sorgulama (polling) ile
 *
 *   bsp_can_init_loopback()   iç geri döngü (TEST.LBCK + CCCR.MON). Gönderilen çerçeve
 *                             denetleyicinin kendi alıcısına içeriden döner; transceiver ve
 *                             pin ayarı gerekmez. Filtre yok: tüm standart kimlikler kabul.
 *   bsp_can_init_loopback_filtered()  aynı, ama normal moddaki kabul filtresiyle (filtre testi)
 *   bsp_can_init_normal()     gerçek hat: PA11 = FDCAN1_RX, PA12 = FDCAN1_TX (AF9), harici
 *                             transceiver gerekir. TEST/LBCK/MON kapalı. Kabul filtresi:
 *                             yalnızca 0x101, 0x102, 0x301 ve 0x110-0x114 (diğer std/ext ve uzak
 *                             çerçeveler reddedilir).
 *
 * Bus-off: denetleyici bus-off olunca CCCR.INIT'i kendisi 1 yapar ve hattan çekilir.
 * bsp_can_service(now_ms) periyodik çağrılır (ör. her 10 ms). Bus-off görürse bekleme süresi
 * dolunca INIT'i temizler; denetleyici ayrıca 129 x 11 resesif bit (500 kbit/s'de ~2,8 ms)
 * bekleyip hatta döner. Bekleme: ardışık ilk BSP_CAN_BO_FAST_N kurtarmada BSP_CAN_BO_FAST_MS,
 * sonra BSP_CAN_BO_SLOW_MS (kalıcı arızada hattı hata çerçeveleriyle boğmamak için).
 * Son kurtarmadan BSP_CAN_BO_SLOW_MS sonra gelen bus-off yeni bir olay sayılır (hızlı moda döner).
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H
#include <stdint.h>

#define BSP_CAN_BITRATE  500000u

#define BSP_CAN_BO_FAST_MS   100u
#define BSP_CAN_BO_FAST_N      5u
#define BSP_CAN_BO_SLOW_MS  1000u

/* Kabul filtresindeki kimlikler (normal mod ve filtreli loopback) + hücre gerilimi aralığı */
#define BSP_CAN_FILTER_IDS  { 0x101u, 0x102u, 0x301u }
#define BSP_CAN_FILTER_RANGE_LO  0x110u
#define BSP_CAN_FILTER_RANGE_HI  0x114u

typedef struct {
    /* anlık durum (ECR, PSR) */
    uint8_t  tec;          /* gönderme hata sayacı 0..255 */
    uint8_t  rec;          /* alma hata sayacı 0..127 */
    uint8_t  rp;           /* REC error passive seviyesine ulaştı (>= 128) */
    uint8_t  ew;           /* uyarı: TEC veya REC >= 96 */
    uint8_t  ep;           /* error passive */
    uint8_t  bo;           /* bus-off (kurtarma sürüyorsa da 1) */
    uint8_t  lec;          /* son hata kodu: 0 yok, 1 stuff, 2 form, 3 ack, 4 bit1, 5 bit0, 6 crc */
    uint8_t  init;         /* CCCR.INIT: 1 ise denetleyici hatta değil */
    /* birikimli sayaçlar (bsp_can_service ve bsp_can_errors toplar) */
    uint32_t cel;          /* protokol hatası sayısı (ECR.CEL toplamı) */
    uint32_t n_busoff;     /* bus-off olayı sayısı */
    uint32_t n_recover;    /* INIT temizlenerek başlatılan kurtarma sayısı */
    uint32_t n_rx_lost;    /* Rx FIFO 0 doluyken kaybedilen çerçeve olayı */
} bsp_can_err_t;

void bsp_can_init_loopback(void);
void bsp_can_init_loopback_filtered(void);
void bsp_can_init_normal(void);

/* 1: çerçeve Tx FIFO'ya kondu, 0: FIFO dolu ya da denetleyici bus-off/INIT'te */
int  bsp_can_send(uint16_t id, const uint8_t *data, uint8_t len);
/* 1: Rx FIFO 0'dan bir çerçeve okundu, 0: FIFO boş */
int  bsp_can_recv(uint16_t *id, uint8_t *data, uint8_t *len);

/* Periyodik bakım: bus-off kurtarma (bekleme süreli) ve hata olaylarının toplanması.
 * now_ms: uygulamanın milisaniye sayacı. Dönüş: 1 = bu çağrıda bus-off kurtarması başlatıldı. */
int  bsp_can_service(uint32_t now_ms);
void bsp_can_errors(bsp_can_err_t *e);
/* Yazılım sayaçlarını sıfırla (cel, n_busoff, n_recover, n_rx_lost); TEC/REC donanımındır, dokunulmaz */
void bsp_can_clear_stats(void);

#endif
