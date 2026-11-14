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
#define CAN_ID_PC_CMD      0x301u

#define CAN_TIMEOUT_FAULT_MS  250u   /* 2 periyot kaçırıldı -> arıza bayrağı */
#define CAN_TIMEOUT_SAFE_MS   500u   /* 5 periyot -> SAFE-STOP (CAN_ok = 0) */
/* Açılış: ilk geçerli mesaj gelene kadar zaman aşımı bu süreyle değerlendirilir (diğer düğümler
 * açılırken erken SAFE'e kilitlenmemek için; AUTOSAR COM'daki "first timeout" fikri). Süre
 * dolduğunda hâlâ mesaj yoksa doğrudan fault + SAFE. İlk mesajdan sonra 250/500 ms geçerlidir. */
#define CAN_TIMEOUT_FIRST_MS 2000u

typedef struct { float V, I, T; } can_meas1_t;
typedef struct { float V_min, V_max, T_amb; } can_meas2_t;
typedef struct { float I_allow_dis, I_allow_chg; uint8_t state, cause; } can_ctrl_t;
typedef struct { float soc_pct, tc_hat, soh_pct, psi; } can_status_t;
typedef struct { uint8_t cmd; int16_t value; } can_cmd_t;

/* CRC-8 SAE J1850 (poly 0x1D, init 0xFF, xorout 0xFF); "123456789" -> 0x4B */
uint8_t crc8_j1850(const uint8_t *data, int len);

/* Kodlama: ölçek + yuvarlama + doyma, sayaç ve CRC yazılır. */
void can_encode_meas1(const can_meas1_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_meas2(const can_meas2_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_ctrl(const can_ctrl_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_status(const can_status_t *m, uint8_t counter, uint8_t d[8]);
void can_encode_cmd(const can_cmd_t *m, uint8_t counter, uint8_t d[8]);

/* Çözme: CRC doğruysa 1 döner (sayaç *counter'a yazılır), değilse 0 ve çıktı değişmez. */
int can_decode_meas1(const uint8_t d[8], can_meas1_t *m, uint8_t *counter);
int can_decode_meas2(const uint8_t d[8], can_meas2_t *m, uint8_t *counter);
int can_decode_ctrl(const uint8_t d[8], can_ctrl_t *m, uint8_t *counter);
int can_decode_status(const uint8_t d[8], can_status_t *m, uint8_t *counter);
int can_decode_cmd(const uint8_t d[8], can_cmd_t *m, uint8_t *counter);

/* Bir mesaj kimliği için alım izleyicisi: CRC hatası, sayaç atlaması/donması, zaman aşımı. */
typedef struct {
    uint32_t last_ok_ms;        /* son geçerli mesaj zamanı */
    uint8_t  last_counter, have_counter;
    uint32_t n_ok, n_crc_err, n_cnt_err, n_lost;
    uint8_t  fault, safe;       /* fault: zaman aşımı >= 250 ms; safe: >= 500 ms (kalıcı) */
} can_monitor_t;
void can_monitor_init(can_monitor_t *mon, uint32_t now_ms);
/* Her alınan çerçevede: crc_ok ve sayaç ile çağrılır. Sayaç beklenen değilse (donma veya atlama)
 * çerçeve reddedilir (0 döner); kayıp çerçeve sayısı n_lost'a eklenir. */
int  can_monitor_rx(can_monitor_t *mon, uint32_t now_ms, int crc_ok, uint8_t counter);
/* Periyodik (ör. her 10 ms): zaman aşımı kontrolü. CAN_ok = !mon->safe
 * İlk geçerli mesajdan önce eşik CAN_TIMEOUT_FIRST_MS'dir (init anından itibaren). */
void can_monitor_tick(can_monitor_t *mon, uint32_t now_ms);

#endif
