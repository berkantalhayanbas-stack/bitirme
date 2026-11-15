/* bsp_safety.c -- IWDG, reset nedeni, flash imajı CRC kontrolü (bkz. bsp_safety.h). */
#include "stm32g4xx.h"
#include "bsp_safety.h"

/* ------------------------------------------------------------------ reset nedeni */
bsp_reset_t bsp_reset_cause(void)
{
    const uint32_t csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;                             /* bir sonraki reset için temizle */
    if (csr & RCC_CSR_IWDGRSTF)  return RST_IWDG;
    if (csr & RCC_CSR_WWDGRSTF)  return RST_WWDG;
    if (csr & RCC_CSR_SFTRSTF)   return RST_SOFTWARE;
    if (csr & RCC_CSR_LPWRRSTF)  return RST_LOWPOWER;
    if (csr & RCC_CSR_OBLRSTF)   return RST_OPTION_BYTES;
    if (csr & RCC_CSR_BORRSTF)   return RST_POWER;
    return RST_PIN;
}

const char *bsp_reset_name(bsp_reset_t r)
{
    static const char *const n[] = { "power-on/BOR", "NRST pin", "software", "WDG reset (IWDG)",
                                     "WWDG", "low-power", "option bytes" };
    return (unsigned)r < sizeof(n) / sizeof(n[0]) ? n[r] : "?";
}

/* ------------------------------------------------------------------ IWDG (~100 ms) */
void bsp_iwdg_start(void)
{
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;   /* hata ayıklayıcı çekirdeği durdurunca dur */
    IWDG->KR  = 0xCCCCu;                                  /* başlat (LSI otomatik açılır) */
    IWDG->KR  = 0x5555u;                                  /* PR/RLR yazma izni */
    IWDG->PR  = 0u;                                       /* /4 -> 8 kHz */
    IWDG->RLR = BSP_IWDG_TIMEOUT_MS * 8u;                 /* 800 -> 100 ms */
    while (IWDG->SR != 0u) { }                            /* kayıtlar güncellensin */
    IWDG->KR  = 0xAAAAu;
}

void bsp_iwdg_kick(void) { IWDG->KR = 0xAAAAu; }

/* ------------------------------------------------------------------ flash CRC */
extern uint32_t _sidata, _sdata, _edata;                  /* bağlayıcı betiği (STM32G431RBTX_FLASH.ld) */

bsp_crc_t bsp_flash_crc_check(uint32_t *crc_out, uint32_t *len_out)
{
    const uint32_t *ref = (const uint32_t *)BSP_CRC_REF_ADDR;
    /* imaj = flash başı .. .data'nın yükleme kopyasının sonu */
    const uint32_t len = (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata) - FLASH_BASE;
    const uint32_t words = (len + 3u) / 4u;
    const uint32_t *p = (const uint32_t *)FLASH_BASE;

    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    (void)RCC->AHB1ENR;
    CRC->INIT = 0xFFFFFFFFu;
    CRC->POL  = 0x04C11DB7u;
    CRC->CR   = CRC_CR_RESET;                             /* 32 bit, yansıtma yok */
    for (uint32_t i = 0; i < words; i++) CRC->DR = p[i];
    const uint32_t crc = CRC->DR;

    if (crc_out) *crc_out = crc;
    if (len_out) *len_out = len;
    if (ref[0] != BSP_CRC_REF_MAGIC) return CRC_NO_REF;
    if (ref[1] != len)               return CRC_LEN_MISMATCH;
    return ref[2] == crc ? CRC_OK : CRC_MISMATCH;
}

const char *bsp_crc_name(bsp_crc_t c)
{
    return c == CRC_OK ? "OK" : c == CRC_NO_REF ? "no reference" : c == CRC_LEN_MISMATCH ? "length mismatch"
                                                                                         : "CRC MISMATCH";
}
