/* mcp2515_bridge.ino -- Arduino Uno + MCP2515, PC <-> CAN köprüsü (kayıtlı veriyi CAN'dan oynatma).
 *
 * PC'deki tools/pil_test/can_replay.py kayıtlı ölçüm dizisini (data/<senaryo>_in.csv) örnek örnek
 * yollar; köprü her örneği 0x101 + 0x102 olarak CAN'a basar, ECU'nun (APP_ECU) o örnek için
 * gönderdiği 0x201 + 0x202'yi bekler, çözer ve PC'ye döndürür. Kilitli adım (lock-step):
 * PC cevabı almadan sonraki örneği yollamaz, bu yüzden hız serbesttir (gerçek zamandan hızlı).
 *
 * Kütüphane : "mcp2515" (autowp).  Bağlantı: CS = D10, SPI D11/D12/D13, 8 MHz kristal, 500 kbit/s
 * Kodlama/çözme: bms_can.c / bms_can.h = projedeki bms/common/'ın birebir kopyası.
 *
 * Seri protokol (115200 8N1, ikili, little-endian, CRC-8 SAE J1850; PIL protokolüyle aynı kalıp)
 *   PC -> köprü, 28 bayt: A5 5A | cmd (1) | 6 x float32: I V T V_min V_max T_amb | crc(cmd..payload)
 *       cmd 0 = örnek (CAN'a gönder, cevabı bekle), cmd 1 = merhaba (CAN alımını boşalt, durum döndür)
 *   köprü -> PC, 30 bayt: 5A A5 | status (1) | I_dis (f32) | I_chg (f32) | state (1) | cause (1)
 *                         | soc_pct (f32) | tc (f32) | soh_pct (f32) | psi (f32) | crc(status..psi)
 *       status bit0 geçerli 0x201 alındı      bit1 geçerli 0x202 alındı
 *              bit2 seri çerçevede CRC hatası bit3 CAN gönderim hatası (Tx tamponu dolu)
 *              bit4 örnekten önce bekleyen 0x201/0x202 vardı (ECU kendi başına adım atmış)
 *              bit5 CRC'si bozuk CAN çerçevesi alındı
 *              bit7 merhaba cevabı
 */
#include <SPI.h>
#include <mcp2515.h>
extern "C" {
#include "bms_can.h"
}

const uint8_t  PIN_CS     = 10;
const uint32_t RX_WAIT_MS = 100;    /* ECU cevabı için bekleme (ECU'nun 1 s'lik UART raporu ~20 ms sürer) */

enum { ST_CTRL = 0x01, ST_STAT = 0x02, ST_SER_CRC = 0x04, ST_TX_ERR = 0x08,
       ST_STALE = 0x10, ST_CAN_CRC = 0x20, ST_HELLO = 0x80 };

MCP2515 mcp2515(PIN_CS);
uint8_t cnt_tx = 0;

/* ---------------------------------------------------------------- CAN */
bool sendRaw(uint16_t id, const uint8_t d[8])
{
  struct can_frame f;
  f.can_id  = id;
  f.can_dlc = 8;
  memcpy(f.data, d, 8);
  return mcp2515.sendMessage(&f) == MCP2515::ERROR_OK;
}

/* Bekleyen bütün çerçeveleri at; kaç tane olduğunu döndür */
uint8_t flushCan()
{
  struct can_frame f;
  uint8_t n = 0;
  while (mcp2515.readMessage(&f) == MCP2515::ERROR_OK) n++;
  if (mcp2515.getErrorFlags() & 0xC0) mcp2515.clearRXnOVR();   /* EFLG bit7/6: RX1OVR / RX0OVR */
  return n;
}

/* ---------------------------------------------------------------- seri */
void reply(uint8_t status, const can_ctrl_t &c, const can_status_t &s)
{
  uint8_t tx[30] = { 0x5A, 0xA5, status };
  memcpy(&tx[3],  &c.I_allow_dis, 4);
  memcpy(&tx[7],  &c.I_allow_chg, 4);
  tx[11] = c.state;
  tx[12] = c.cause;
  memcpy(&tx[13], &s.soc_pct, 4);
  memcpy(&tx[17], &s.tc_hat, 4);
  memcpy(&tx[21], &s.soh_pct, 4);
  memcpy(&tx[25], &s.psi, 4);
  tx[29] = crc8_j1850(&tx[2], 27);
  Serial.write(tx, sizeof(tx));
}

/* ---------------------------------------------------------------- kurulum / döngü */
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(50);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop()
{
  uint8_t rx[26];                                   /* cmd + 24 bayt payload + crc */
  can_ctrl_t   ctrl = { 0, 0, 0, 0 };
  can_status_t stat = { 0, 0, 0, 0 };

  /* ---- senkronizasyon: A5 5A ara ---- */
  if (Serial.available() < 1 || Serial.read() != 0xA5) return;
  if (Serial.readBytes(rx, 1) != 1 || rx[0] != 0x5A) return;
  if (Serial.readBytes(rx, sizeof(rx)) != sizeof(rx)) return;
  if (crc8_j1850(rx, 25) != rx[25]) { reply(ST_SER_CRC, ctrl, stat); return; }

  if (rx[0] == 1) {                                 /* merhaba */
    flushCan();
    reply(ST_HELLO, ctrl, stat);
    return;
  }

  float v[6];                                       /* I V T V_min V_max T_amb */
  memcpy(v, &rx[1], sizeof(v));
  uint8_t status = flushCan() ? ST_STALE : 0;

  const can_meas1_t m1 = { v[1], v[0], v[2] };      /* V, I, T */
  const can_meas2_t m2 = { v[3], v[4], v[5] };      /* V_min, V_max, T_amb */
  uint8_t d[8];
  can_encode_meas1(&m1, cnt_tx, d);
  if (!sendRaw(CAN_ID_EMU_MEAS1, d)) status |= ST_TX_ERR;
  can_encode_meas2(&m2, cnt_tx, d);
  if (!sendRaw(CAN_ID_EMU_MEAS2, d)) status |= ST_TX_ERR;
  cnt_tx = (cnt_tx + 1) & 0x0F;

  /* ---- ECU'nun bu örnek için gönderdiği 0x201 + 0x202'yi bekle ---- */
  const uint32_t t0 = millis();
  while ((status & (ST_CTRL | ST_STAT)) != (ST_CTRL | ST_STAT) && millis() - t0 < RX_WAIT_MS) {
    struct can_frame f;
    if (mcp2515.readMessage(&f) != MCP2515::ERROR_OK) continue;
    if (f.can_dlc != 8 || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG))) continue;
    const uint16_t id = f.can_id & CAN_SFF_MASK;
    uint8_t c;
    if (id == CAN_ID_ECU_CTRL) {
      if (can_decode_ctrl(f.data, &ctrl, &c)) status |= ST_CTRL; else status |= ST_CAN_CRC;
    } else if (id == CAN_ID_ECU_STATUS) {
      if (can_decode_status(f.data, &stat, &c)) status |= ST_STAT; else status |= ST_CAN_CRC;
    }
  }
  reply(status, ctrl, stat);
}
