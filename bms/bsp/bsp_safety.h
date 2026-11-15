/* bsp_safety.h -- fonksiyonel güvenlik yardımcıları: bağımsız watchdog (IWDG), reset nedeni,
 * açılışta flash imajının CRC kontrolü (donanım CRC birimi). Kayıt seviyesinde, HAL yok.
 *
 * Flash CRC referansı: imajın kendisinde değil, son flash sayfasında (0x0801F800) durur:
 *   { magic 'CRC1', uzunluk [bayt], crc32 }  -- tools/fw_build.py --crc-ref yazar.
 * CRC: STM32 CRC birimi varsayılanı (poly 0x04C11DB7, init 0xFFFFFFFF, yansıtma yok, 32 bit kelime;
 * CRC-32/MPEG-2), imaj başından (0x08000000) uzunluk kadar (4'e yuvarlanmış, sayfanın kalanı 0xFF).
 */
#ifndef BSP_SAFETY_H
#define BSP_SAFETY_H
#include <stdint.h>

#define BSP_IWDG_TIMEOUT_MS   100u
#define BSP_CRC_REF_ADDR      0x0801F800u
#define BSP_CRC_REF_MAGIC     0x31435243u      /* "CRC1" (little-endian) */

typedef enum {                 /* öncelik sırasıyla; NRST pin bayrağı her resette de kalkar */
    RST_POWER = 0,             /* açılış / brown-out */
    RST_PIN,                   /* reset düğmesi, ST-LINK */
    RST_SOFTWARE,              /* NVIC_SystemReset (ör. 0x301 yeniden başlatma komutu) */
    RST_IWDG,                  /* bağımsız watchdog */
    RST_WWDG,
    RST_LOWPOWER,
    RST_OPTION_BYTES
} bsp_reset_t;

bsp_reset_t bsp_reset_cause(void);            /* RCC_CSR'den okur ve bayrakları temizler (açılışta bir kez) */
const char *bsp_reset_name(bsp_reset_t r);

void bsp_iwdg_start(void);                    /* ~100 ms (LSI 32 kHz / 4, reload 800); durdurulamaz */
void bsp_iwdg_kick(void);

typedef enum { CRC_OK = 0, CRC_NO_REF, CRC_LEN_MISMATCH, CRC_MISMATCH } bsp_crc_t;
/* Çalışan imajın uzunluğu (bağlayıcı sembolleri) ile referansı karşılaştırır ve CRC'yi hesaplar.
 * crc_out / len_out: hesaplanan değerler (rapor için). */
bsp_crc_t bsp_flash_crc_check(uint32_t *crc_out, uint32_t *len_out);
const char *bsp_crc_name(bsp_crc_t c);

#endif
