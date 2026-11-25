/* bsp_nvm.h -- kalıcı kayıt (öğrenilmiş SOH), flash'ın 61-62. sayfalarında halka (docs/soh_design.md).
 *
 *   0x0801E800  sayfa 61  \  iki sayfa dönüşümlü: yeni kayıt sona eklenir, sayfa dolunca öbürü silinip
 *   0x0801F000  sayfa 62  /  oradan sürer. Açılışta CRC'si tutan en büyük sıra numaralı kayıt okunur.
 *   0x0801F800  sayfa 63     imaj CRC referansı (bsp_safety), dokunulmaz.
 * Kayıt 48 B (6 çift kelime): sihirli sayı, sıra no, veri (32 B; app_ecu: soh_rec_t), CRC-32 (ilk 40 B), dolgu.
 * Yazma kesilirse (güç) yarım kayıt CRC'den geçmez: bir önceki kayıt kullanılır. Yarım programlanmış çift
 * kelimenin ECC çift hatası NMI'ye düşer; NMI_Handler bunu temizleyip okumayı sürdürür (CRC yine reddeder).
 * Sayfa silme ~22 ms CPU'yu durdurur (tek bank): app_ecu yalnız dinlenmede yazar. STM32_Programmer ile
 * imaj yüklemek yalnız imajın sayfalarını siler: öğrenilen değerler yazılım güncellemesinden sağ çıkar.
 */
#ifndef BSP_NVM_H
#define BSP_NVM_H
#include <stdint.h>

#define BSP_NVM_BASE    0x0801E800u
#define BSP_NVM_PAGE0   61u
#define BSP_NVM_SLOTS   42u          /* 2048 / 48 */
#define BSP_NVM_DATA    32u          /* kayıt başına veri [B] */

typedef struct {
    uint32_t seq, page, slot;        /* son okunan / yazılan kaydın yeri */
    uint32_t n_valid, n_bad;         /* açılış taraması: geçerli / bozuk (dolu ama CRC'siz) kayıt */
    uint8_t  ecc;                    /* taramada ECC çift hatası görüldü */
} bsp_nvm_info_t;

/* en yeni geçerli kaydı okur; 1 = bulundu */
int  bsp_nvm_load(void *data, bsp_nvm_info_t *info);
/* yeni kayıt ekler (gerekirse öbür sayfayı siler); 1 = yazıldı ve geri okununca doğru */
int  bsp_nvm_save(const void *data);
/* iki sayfayı da siler (yeni paket) */
int  bsp_nvm_erase_all(void);
/* imajın sonu NVM alanının altında mı (bağlayıcı betiğine dokunmadan çalışma anı denetimi) */
int  bsp_nvm_layout_ok(void);

#endif
