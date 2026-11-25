/* bms_can.h -- CAN mesajları (bms.dbc), bütünlük koruması (alive counter + CRC-8) ve
 * zaman aşımı izleyicisi. Donanımdan bağımsız: sadece 8 baytlık veri dizileriyle çalışır;
 * STM32 FDCAN ve MCP2515 sürücüleri bu fonksiyonları çağırır.
 */
#ifndef BMS_CAN_H
#define BMS_CAN_H
#include <stdint.h>

#define CAN_ID_EMU_MEAS1   0x101u
#define CAN_ID_EMU_MEAS2   0x102u
#define CAN_ID_ECU_CTRL    0x201u
#define CAN_ID_ECU_STATUS  0x202u
#define CAN_ID_PC_CMD      0x301u     /* olay mesajı: periyodik değil, zaman aşımı izlenmez */

/* Hücre grubu gerilimleri (EMU -> ECU, 100 ms): 0x110 + k, k = 0..4, grup 4k+1 .. 4k+4 (19 grup;
 * 0x114'ün 4. yuvası boş). Düzen: d[0..5] = 4 x 12 bit gerilim (küçük uçlu bit dizisi, yuva j
 * bit 12j'den başlar), V = 1,000 V + ham x 1 mV (1,000..5,094 V), ham 0xFFF = yok / geçersiz;
 * d[6] alt 4 bit sayaç, ÜST 4 bit = bu çerçevedeki grupların dengeleme anahtarı geri bildirimi
 * (bit j = yuva j; AFE'nin gerçekten açtığı); d[7] CRC. Zaman aşımı yalnız dengelemeyi kapatır,
 * CAN_ok'a (SAFE) girmez. */
#define CAN_ID_EMU_CELL0   0x110u
#define CAN_N_CELL_FRAMES  5u
#define CAN_N_CELLS        19u
#define CAN_CELLS_PER_FRAME 4u
/* Dengeleme komutu (ECU -> EMU, her BMS adımında): d[0..2] = 19 bit maske (bit i = grup i+1,
 * 1 = deşarj direncini aç), d[3] = bayraklar (CAN_BAL_F_*), d[4..5] = hücre farkı V_max - V_min
 * [0,1 mV, uint16], d[6] sayaç, d[7] CRC. EMU, 0x210 CAN_TIMEOUT_FAULT_MS içinde gelmezse tüm
 * anahtarları kapatır. */
#define CAN_ID_ECU_BAL     0x210u
enum {
    CAN_BAL_F_ACTIVE  = 0x01,      /* dengeleme koşulları sağlandı (bekleme süresi doldu) */
    CAN_BAL_F_STALE   = 0x02,      /* hücre çerçevelerinden en az biri eski/yok */
    CAN_BAL_F_DRIVE   = 0x04,      /* sürüş akımı var (deşarj) */
    CAN_BAL_F_BLOCKED = 0x08       /* durum LIMP/SAFE, sıcaklık ya da en düşük hücre gerilimi engelliyor */
};

/* 0x301 komutları: d[0] = cmd, d[1..2] = value (int16, little-endian), d[3..5] = 0.
 * Aralıklar: 0x01-0x0F ECU, 0x10-0x1F EMU; her düğüm yalnız kendi aralığını işler.
 * Kabul (olay mesajı): CRC doğru ve alive counter bir önceki kabul edilen komutunkinden FARKLI
 * (aynı çerçevenin tekrarı reddedilir; dizi atlaması kontrol edilmez, çünkü komutlar seyrek ve
 * gönderen yeniden başlayınca sayaç sıfırdan başlar). EMU komutlarında value = süre [ms]
 * (uint16 olarak okunur, 0 = 0x10 gelene kadar sürekli); hepsi 0x101 ve 0x102'ye birlikte uygulanır
 * (twin_emu'da 0x110-0x114 hücre çerçevelerine de). */
enum {
    CAN_CMD_ECU_RESTART   = 0x01,  /* yazılımla yeniden başlat (SAFE'ten çıkış): value = CAN_CMD_UNLOCK,
                                      yalnız taze 0x101'de |I| < CAN_CMD_RESTART_I_A iken; ölçüm
                                      yoksa reddedilir (açılışta gelen bayat komutla art arda
                                      yeniden başlamayı önler) */
    CAN_CMD_ECU_SET_SOC   = 0x02,  /* EKF SOC kestirimini ayarla: value = SOC [0.01 %], 0..10000 */
    CAN_CMD_ECU_CLR_STATS = 0x03,  /* CAN ve uygulama hata sayaçlarını sıfırla */
    CAN_CMD_ECU_HANG      = 0x04,  /* watchdog testi: value = CAN_CMD_UNLOCK -> sonsuz döngü; IWDG
                                      ~100 ms'de sıfırlar, ECU "WDG reset" ile SAFE-STOP'ta açılır.
                                      Yalnız BMS_SAFETY derlemesinde; diğerlerinde bilinmeyen komut */
    CAN_CMD_ECU_NEW_PACK  = 0x05,  /* yeni paket takıldı: value = CAN_CMD_UNLOCK, |I| < CAN_CMD_RESTART_I_A;
                                      öğrenilmiş SOH kaydını (flash) siler, SOH %100'den yeniden öğrenilir */
    CAN_CMD_EMU_CLEAR     = 0x10,  /* tüm enjekte hataları kaldır */
    CAN_CMD_EMU_CRC       = 0x11,  /* CRC'yi boz */
    CAN_CMD_EMU_FREEZE    = 0x12,  /* alive counter'ı dondur */
    CAN_CMD_EMU_SILENT    = 0x13,  /* 0x101/0x102 gönderme (kablo kopması taklidi) */
    CAN_CMD_EMU_NTC_OPEN  = 0x14   /* NTC kopuk: T = -40 °C gönder */
};
#define CAN_CMD_UNLOCK        ((int16_t)0xA5A5)   /* yeniden başlatma kilidi */
#define CAN_CMD_RESTART_I_A   1.0f                /* yeniden başlatmaya izin veren akım sınırı [A] */
/* Açılıştan sonra bu süre boyunca ECU tüm 0x301 komutlarını reddeder: ECU kapalıyken gönderilen
 * çerçeveler ACK alamadığı için gönderenin Tx tamponunda bekler ve ECU hatta katılınca teslim edilir.
 * Bu bayat çerçeveler arasında 0x101 (yüksek öncelik) komuttan önce gelir; bekletilmiş bir yeniden
 * başlatma komutu, bekletilmiş bir |I| ~ 0 ölçümüyle birlikte kabul edilip ECU'yu art arda
 * yeniden başlatırdı (kartta görüldü). */
#define CAN_CMD_HOLDOFF_MS    1000u

#define CAN_TIMEOUT_FAULT_MS  250u   /* 2 periyot kaçırıldı -> arıza bayrağı */
#define CAN_TIMEOUT_SAFE_MS   500u   /* son geçerli mesajdan 500 ms -> SAFE-STOP (CAN_ok = 0) */
/* Sayaç kuralıyla birlikte (bms_can.c MAX_DELTA = 3) 100 ms periyotta etkin tolerans:
 *   1-2 ardışık kayıp : dönüş çerçevesi kabul (n_lost), yalnız fault
 *   3 kayıp           : dönüş çerçevesi reddedilir (sayaç farkı 4), sonraki kabul; 400 ms'de
 *                       değerlendirmede yaş 497 ms < 500 -> SAFE yok (sınırda)
 *   4 ve üstü kayıp   : dönüş çerçevesi reddedilir (fark >= 5) -> yaş 500 ms'yi geçer -> SAFE
 * Yani pratikte 4 ardışık kayıp SAFE-STOP'a götürür (AUTOSAR E2E "wrong sequence": sıra hatalı
 * çerçevenin verisi kullanılmaz). Bilinçli seçim; simulink/can_rx.m ve twin/can_monitor.py
 * aynı davranışı modeller ve test eder (twin/test_can_monitor.py U3, U9). */
/* Açılış: ilk geçerli mesaj gelene kadar zaman aşımı bu süreyle değerlendirilir (diğer düğümler
 * açılırken erken SAFE'e kilitlenmemek için; AUTOSAR COM'daki "first timeout" fikri). Süre
 * dolduğunda hâlâ mesaj yoksa doğrudan fault + SAFE. İlk mesajdan sonra 250/500 ms geçerlidir. */
#define CAN_TIMEOUT_FIRST_MS 2000u

typedef struct { float V, I, T; } can_meas1_t;
typedef struct { float V_min, V_max, T_amb; } can_meas2_t;
typedef struct { float I_allow_dis, I_allow_chg; uint8_t state, cause; } can_ctrl_t;
typedef struct { float soc_pct, tc_hat, soh_pct, psi; } can_status_t;
typedef struct { uint8_t cmd; int16_t value; } can_cmd_t;
typedef struct { float v[CAN_CELLS_PER_FRAME]; uint8_t valid, bal; } can_cell_t;   /* valid/bal: bit j = yuva j */
typedef struct { uint32_t mask; uint8_t flags; float dv; } can_bal_t;              /* dv [V] */

/* CRC-8 SAE J1850 (poly 0x1D, init 0xFF, xorout 0xFF); "123456789" -> 0x4B */
uint8_t crc8_j1850(const uint8_t *data, int len);

/* Kodlama: ölçek + yuvarlama + doyma, sayaç ve CRC yazılır. */
void can_encode_meas1(const can_meas1_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_meas2(const can_meas2_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_ctrl(const can_ctrl_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_status(const can_status_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_cmd(const can_cmd_t *m, uint8_t counter, uint8_t d[8]);
/* k = çerçeve sırası 0..4 (kimlik CAN_ID_EMU_CELL0 + k); valid'de biti 0 olan yuva 0xFFF yazılır */
void can_encode_cell(uint8_t k, const can_cell_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_bal(const can_bal_t *m, uint8_t counter, uint8_t d[8]);

/* Çözme: CRC doğruysa 1 döner (sayaç *counter'a yazılır), değilse 0 ve çıktı değişmez. */
int can_decode_meas1(const uint8_t d[8], can_meas1_t *m, uint8_t *counter);
int can_decode_meas2(const uint8_t d[8], can_meas2_t *m, uint8_t *counter);
int can_decode_ctrl(const uint8_t d[8], can_ctrl_t *m, uint8_t *counter);
int can_decode_status(const uint8_t d[8], can_status_t *m, uint8_t *counter);
int can_decode_cmd(const uint8_t d[8], can_cmd_t *m, uint8_t *counter);
int can_decode_cell(uint8_t k, const uint8_t d[8], can_cell_t *m, uint8_t *counter);
int can_decode_bal(const uint8_t d[8], can_bal_t *m, uint8_t *counter);

/* Bir mesaj kimliği için alım izleyicisi: CRC hatası, sayaç atlaması/donması, zaman aşımı. */
typedef struct {
    uint32_t last_ok_ms;        /* son geçerli mesaj zamanı */
    uint8_t  last_counter, have_counter;
    uint32_t n_ok, n_crc_err, n_cnt_err, n_lost;
    uint8_t  fault, safe;       /* fault: zaman aşımı >= 250 ms; safe: >= 500 ms (kalıcı) */
} can_monitor_t;
void can_monitor_init(can_monitor_t *mon, uint32_t now_ms);
/* Her alınan çerçevede: crc_ok ve sayaç ile çağrılır. Sayaç farkı 1 normal; 2..MAX_DELTA arada
 * kayıp çerçeve var ama kabul (n_lost'a eklenir); 0 (donma/tekrar) veya > MAX_DELTA (dizi hatası)
 * reddedilir (0 döner, n_cnt_err), sayaç yeniden eşitlenir. CRC hatası da reddedilir. */
int  can_monitor_rx(can_monitor_t *mon, uint32_t now_ms, int crc_ok, uint8_t counter);
/* Periyodik (ör. her 10 ms): zaman aşımı kontrolü. CAN_ok = !mon->safe
 * Yalnız periyodik mesajlar (0x101, 0x102) için çağrılır; olay mesajı 0x301 için çağrılmaz.
 * İlk geçerli mesajdan önce eşik CAN_TIMEOUT_FIRST_MS'dir (init anından itibaren). */
void can_monitor_tick(can_monitor_t *mon, uint32_t now_ms);

#endif
