/* mcp2515_node.ino -- Arduino Uno + MCP2515, gerçek CAN hattında "EMU" düğümü (BMS ECU'nun karşısı).
 *
 * Kütüphane : "mcp2515" (autowp), Library Manager'dan
 * Bağlantı  : CS = D10, INT = D2, SPI: D11 MOSI, D12 MISO, D13 SCK, VCC 5 V, GND
 * CAN       : 500 kbit/s, modül kristali 8 MHz, NORMAL mod (CANH/CANL hatta bağlı)
 *
 * Her 100 ms : 0x101 (V, I, T) ve 0x102 (V_min, V_max, T_amb) gönder
 * Sürekli    : 0x201 (ECU_CTRL) ve 0x202 (ECU_STATUS) al, çöz, CRC + sayaç izleyicisinden geçir
 * Her 1 s    : Serial'a (115200) tek satır rapor
 *
 * Ölçümler basit bir paket taklididir (gerçek emülatör değil): akım -50..+50 A arasında yavaşça
 * salınır, V = 70,3 V - I x R_paket, sıcaklığa 0,1 °C'lik ölçüm gürültüsü eklenir (ECU'nun
 * sensör teşhisi 5 s boyunca birebir aynı gelen sıcaklığı "donmuş sensör" sayar).
 *
 * Kodlama/çözme, firmware ile AYNI kod: bms_can.c / bms_can.h, projedeki bms/common/'dan
 * kopyalandı (Arduino derleyicisi yalnız sketch klasöründeki dosyaları görür).
 * bms/common/bms_can.* değişirse bu kopyalar da güncellenmeli.
 */
#include <SPI.h>
#include <mcp2515.h>
extern "C" {
#include "bms_can.h"
}

const uint8_t  PIN_CS    = 10;
const uint8_t  PIN_INT   = 2;
const uint32_t TX_PERIOD = 100;    /* ms, CAN 0x101/0x102 periyodu (BMS_TS = 0,1 s) */
const uint32_t REPORT    = 1000;   /* ms */

MCP2515 mcp2515(PIN_CS);

uint8_t  cnt_tx = 0;
uint32_t t_tx = 0, t_rep = 0;
uint32_t n_tx_ok = 0, n_tx_err = 0, n_rx_other = 0;
can_monitor_t mon_status, mon_ctrl;
can_status_t  last_status;
can_ctrl_t    last_ctrl;
uint8_t       have_status = 0, have_ctrl = 0;

/* ---------------------------------------------------------------- gönderme */
bool sendRaw(uint16_t id, const uint8_t d[8])
{
  struct can_frame f;
  f.can_id  = id;
  f.can_dlc = 8;
  memcpy(f.data, d, 8);
  return mcp2515.sendMessage(&f) == MCP2515::ERROR_OK;
}

void sendMeasurements(uint32_t now)
{
  /* örnek paket değerleri: akım yavaşça -50..+50 A arasında salınır */
  const float phase = (float)(now % 20000UL) / 20000.0f;
  const float I = (phase < 0.5f) ? (-50.0f + 200.0f * phase) : (150.0f - 200.0f * phase);
  const float V = 70.3f - 0.0418f * I;             /* R_paket = 26,4 mOhm x 19 / 12 */
  const float T = 25.4f + ((cnt_tx & 1) ? 0.1f : 0.0f);   /* ölçüm gürültüsü taklidi */
  const can_meas1_t m1 = { V, I, T };
  const can_meas2_t m2 = { V / 19.0f - 0.010f, V / 19.0f + 0.010f, 22.0f };
  uint8_t d[8];

  can_encode_meas1(&m1, cnt_tx, d);
  bool ok1 = sendRaw(CAN_ID_EMU_MEAS1, d);
  can_encode_meas2(&m2, cnt_tx, d);
  bool ok2 = sendRaw(CAN_ID_EMU_MEAS2, d);

  n_tx_ok  += ok1 + ok2;
  n_tx_err += !ok1 + !ok2;
  cnt_tx = (cnt_tx + 1) & 0x0F;                    /* 4 bit alive counter */
}

/* ---------------------------------------------------------------- alma */
void receiveAll(uint32_t now)
{
  struct can_frame f;
  while (mcp2515.readMessage(&f) == MCP2515::ERROR_OK) {
    const uint16_t id = f.can_id & CAN_SFF_MASK;
    uint8_t c = 0;
    if (f.can_dlc != 8 || (f.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG))) {
      n_rx_other++;
    } else if (id == CAN_ID_ECU_STATUS) {
      can_status_t st;
      const int crc_ok = can_decode_status(f.data, &st, &c);
      if (can_monitor_rx(&mon_status, now, crc_ok, c)) { last_status = st; have_status = 1; }
    } else if (id == CAN_ID_ECU_CTRL) {
      can_ctrl_t ct;
      const int crc_ok = can_decode_ctrl(f.data, &ct, &c);
      if (can_monitor_rx(&mon_ctrl, now, crc_ok, c)) { last_ctrl = ct; have_ctrl = 1; }
    } else {
      n_rx_other++;
    }
  }
}

/* ---------------------------------------------------------------- rapor */
void report(uint32_t now)
{
  Serial.print(F("t="));      Serial.print(now / 1000);
  Serial.print(F("s | tx ok:")); Serial.print(n_tx_ok);
  Serial.print(F(" err:"));    Serial.print(n_tx_err);
  Serial.print(F(" | 0x201 ok:")); Serial.print(mon_ctrl.n_ok);
  Serial.print(F(" crc:"));    Serial.print(mon_ctrl.n_crc_err);
  Serial.print(F(" cnt:"));    Serial.print(mon_ctrl.n_cnt_err);
  if (mon_ctrl.safe)       Serial.print(F(" !SAFE"));
  else if (mon_ctrl.fault) Serial.print(F(" !TO"));
  if (have_ctrl) {
    static const char *const names[] = { "?", "NORMAL", "DERATE", "LIMP", "SAFE" };
    Serial.print(' ');           Serial.print(names[last_ctrl.state <= 4 ? last_ctrl.state : 0]);
    Serial.print(F(" Idis "));   Serial.print(last_ctrl.I_allow_dis, 1);
    Serial.print(F(" Ichg "));   Serial.print(last_ctrl.I_allow_chg, 1);
    Serial.print(F(" cause "));  Serial.print(last_ctrl.cause);
  }
  Serial.print(F(" | 0x202 ok:")); Serial.print(mon_status.n_ok);
  Serial.print(F(" crc:"));    Serial.print(mon_status.n_crc_err);
  Serial.print(F(" cnt:"));    Serial.print(mon_status.n_cnt_err);
  Serial.print(F(" lost:"));   Serial.print(mon_status.n_lost);
  Serial.print(F(" other:"));  Serial.print(n_rx_other);
  if (mon_status.safe)       Serial.print(F(" !SAFE"));
  else if (mon_status.fault) Serial.print(F(" !TO"));
  if (have_status) {
    Serial.print(F(" | SOC "));  Serial.print(last_status.soc_pct, 2);
    Serial.print(F("% Tc "));    Serial.print(last_status.tc_hat, 1);
    Serial.print(F(" SOH "));    Serial.print(last_status.soh_pct, 1);
    Serial.print(F(" psi "));    Serial.print(last_status.psi, 3);
  }
  Serial.print(F(" | TEC "));  Serial.print(mcp2515.errorCountTX());
  Serial.print(F(" REC "));    Serial.print(mcp2515.errorCountRX());
  Serial.print(F(" EFLG 0x")); Serial.println(mcp2515.getErrorFlags(), HEX);
}

/* ---------------------------------------------------------------- kurulum / döngü */
void setup()
{
  Serial.begin(115200);
  pinMode(PIN_INT, INPUT_PULLUP);
  Serial.println();
  Serial.println(F("=== MCP2515 EMU node, 500 kbit/s, normal mode: TX 0x101/0x102, RX 0x202 ==="));

  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println(F("setBitrate FAILED (SPI baglantisi / CS pini?)"));
    while (1) { }
  }
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println(F("setNormalMode FAILED"));
    while (1) { }
  }
  can_monitor_init(&mon_status, millis());
  can_monitor_init(&mon_ctrl, millis());
  t_tx = t_rep = millis();
}

void loop()
{
  const uint32_t now = millis();

  if (digitalRead(PIN_INT) == LOW) receiveAll(now);

  if ((int32_t)(now - t_tx) >= 0) {
    t_tx += TX_PERIOD;
    receiveAll(now);                    /* yedek: INT bir hata bayrağı yüzünden düşük kalsa da alım sürsün */
    sendMeasurements(now);
    can_monitor_tick(&mon_status, now);
    can_monitor_tick(&mon_ctrl, now);
  }
  if ((int32_t)(now - t_rep) >= 0) {
    t_rep += REPORT;
    report(now);
  }
}
