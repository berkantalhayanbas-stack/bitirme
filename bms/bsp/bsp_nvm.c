/* bsp_nvm.c -- flash'ta kalıcı kayıt halkası, bkz. bsp_nvm.h. Register seviyesinde (RM0440 bölüm 5). */
#include <stddef.h>
#include <string.h>
#include "stm32g4xx.h"
#include "bsp_nvm.h"

#define NVM_MAGIC   0x534F4831u          /* "SOH1" */
#define REC_SIZE    48u
#define PAGE_SIZE   2048u

typedef struct {
    uint32_t magic, seq;
    uint8_t  d[BSP_NVM_DATA];
    uint32_t crc, pad;
} nvm_rec_t;

static uint32_t cur_page, next_slot, last_seq;   /* yazma konumu (load sonrası geçerli) */
static volatile uint8_t ecc_hit;

/* ---------------------------------------------------------------- ECC çift hatası (NMI) */
void NMI_Handler(void)
{
    if (FLASH->ECCR & FLASH_ECCR_ECCD) {
        FLASH->ECCR |= FLASH_ECCR_ECCD;          /* 1 yazınca temizlenir */
        ecc_hit = 1u;
        return;                                  /* okuma çöp veriyle sürer, CRC reddeder */
    }
    for (;;) { }                                 /* başka NMI: beklenmiyor (IWDG sıfırlar) */
}

static uint32_t crc32(const uint8_t *p, uint32_t n)
{   /* CRC-32 (IEEE, yansıtılmış), bit bit: kayıt başına 40 B, hız önemsiz */
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static const nvm_rec_t *slot_ptr(uint32_t page, uint32_t slot)
{
    return (const nvm_rec_t *)(BSP_NVM_BASE + page * PAGE_SIZE + slot * REC_SIZE);
}

static int blank(const nvm_rec_t *p)
{
    const uint32_t *w = (const uint32_t *)p;
    for (uint32_t k = 0; k < REC_SIZE / 4u; k++) if (w[k] != 0xFFFFFFFFu) return 0;
    return 1;
}

static int valid(const nvm_rec_t *p)
{
    nvm_rec_t c;
    memcpy(&c, p, sizeof(c));                    /* ECC NMI'si buradaki okumada olur */
    return c.magic == NVM_MAGIC && c.crc == crc32((const uint8_t *)&c, offsetof(nvm_rec_t, crc));
}

/* ---------------------------------------------------------------- flash işlemleri */
static void cache_reset(void)
{   /* silme / yazmadan sonra veri önbelleğinde eski içerik kalmasın */
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
}

static int unlock(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR
              | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR
              | FLASH_SR_OPTVERR;
    return !(FLASH->CR & FLASH_CR_LOCK);
}

static void lock(void) { FLASH->CR |= FLASH_CR_LOCK;  cache_reset(); }

static int wait_ok(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { }
    const uint32_t err = FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR
                                      | FLASH_SR_SIZERR | FLASH_SR_PGSERR | FLASH_SR_MISERR);
    FLASH->SR = FLASH_SR_EOP;
    return err == 0u;
}

static int erase_page(uint32_t page)
{
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB) | FLASH_CR_PER | ((BSP_NVM_PAGE0 + page) << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    const int ok = wait_ok();
    FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);
    return ok;
}

static int program(uint32_t addr, const uint32_t *w, uint32_t n_dw)
{
    int ok = 1;
    FLASH->CR |= FLASH_CR_PG;
    for (uint32_t k = 0; k < n_dw && ok; k++) {
        volatile uint32_t *d = (volatile uint32_t *)(addr + 8u * k);
        d[0] = w[2u * k];
        __ISB();
        d[1] = w[2u * k + 1u];                   /* ikinci kelime çift kelime yazımını başlatır */
        ok = wait_ok();
    }
    FLASH->CR &= ~FLASH_CR_PG;
    return ok;
}

/* ---------------------------------------------------------------- arayüz */
int bsp_nvm_layout_ok(void)
{
    extern uint32_t _sidata, _sdata, _edata;
    return (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata) <= BSP_NVM_BASE;
}

int bsp_nvm_load(void *data, bsp_nvm_info_t *info)
{
    uint32_t best_seq = 0u, best_page = 0u, best_slot = 0u;
    int found = 0;
    memset(info, 0, sizeof(*info));
    ecc_hit = 0u;
    for (uint32_t pg = 0; pg < 2u; pg++)
        for (uint32_t s = 0; s < BSP_NVM_SLOTS; s++) {
            const nvm_rec_t *p = slot_ptr(pg, s);
            if (blank(p)) continue;
            if (!valid(p)) { info->n_bad++; continue; }
            info->n_valid++;
            if (!found || p->seq > best_seq) { found = 1;  best_seq = p->seq;  best_page = pg;  best_slot = s; }
        }
    info->ecc = ecc_hit;
    if (found) {
        memcpy(data, slot_ptr(best_page, best_slot)->d, BSP_NVM_DATA);
        cur_page = best_page;  next_slot = best_slot + 1u;  last_seq = best_seq;
    } else {
        cur_page = 0u;  next_slot = 0u;  last_seq = 0u;
    }
    info->seq = last_seq;  info->page = cur_page;  info->slot = found ? best_slot : 0u;
    return found;
}

int bsp_nvm_save(const void *data)
{
    nvm_rec_t c;
    memset(&c, 0xFF, sizeof(c));
    c.magic = NVM_MAGIC;
    c.seq = last_seq + 1u;
    memcpy(c.d, data, BSP_NVM_DATA);
    c.crc = crc32((const uint8_t *)&c, offsetof(nvm_rec_t, crc));
    if (!unlock()) return 0;
    int ok = 1;
    /* boş yuva ara (yarım kalmış yazımı atla); sayfa dolarsa öbür sayfayı silip baştan */
    while (next_slot < BSP_NVM_SLOTS && !blank(slot_ptr(cur_page, next_slot))) next_slot++;
    if (next_slot >= BSP_NVM_SLOTS) {
        cur_page ^= 1u;  next_slot = 0u;
        ok = erase_page(cur_page);
        cache_reset();
    }
    if (ok) ok = program((uint32_t)slot_ptr(cur_page, next_slot), (const uint32_t *)&c, REC_SIZE / 8u);
    lock();
    if (ok) ok = valid(slot_ptr(cur_page, next_slot)) && slot_ptr(cur_page, next_slot)->seq == c.seq;
    if (ok) { last_seq = c.seq;  next_slot++; }
    else next_slot++;                            /* bozuk yuvayı bir dahaki yazımda atla */
    return ok;
}

int bsp_nvm_erase_all(void)
{
    if (!unlock()) return 0;
    const int ok = erase_page(0u) && erase_page(1u);
    lock();
    cur_page = 0u;  next_slot = 0u;  last_seq = 0u;
    return ok;
}
