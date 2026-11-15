/* mcp2515_node.ino -- Arduino Uno + MCP2515, gerçek CAN hattında "EMU" düğümü (BMS ECU'nun karşısı).
 *
 * Kütüphane : "mcp2515" (autowp), Library Manager'dan
 * Bağlantı  : CS = D10, INT = D2, SPI: D11 MOSI, D12 MISO, D13 SCK, VCC 5 V, GND
 * CAN       : 500 kbit/s, modül kristali 8 MHz, NORMAL mod (CANH/CANL hatta bağlı)
 *
 * Her 100 ms : 0x101 (V, I, T) ve 0x102 (V_min, V_max, T_amb) gönder
 * Sürekli    : 0x201 (ECU_CTRL) ve 0x202 (ECU_STATUS) al, çöz, CRC + sayaç izleyicisinden geçir
 * Her 1 s    : Serial'a (115200) tek satır rapor
 * Seri komut : "c <cmd> <value>" satırı (Serial Monitor, satır sonu: Newline), sayılar ondalık
 *              ya da 0x.. onaltılık. Komut 0x301 olarak CAN'a basılır (her komutta sayaç +1);
 *              EMU aralığındaki (0x10-0x1F) komutları düğüm ayrıca kendisi uygular. Hattan gelen
 *              0x301'in EMU komutları da uygulanır. Komut kodları bms_can.h'de:
 *                c 0x02 5000     ECU: EKF SOC = %50.00
 *                c 0x03 0        ECU: hata sayaçlarını sıfırla
 *                c 0x01 0xA5A5   ECU: yeniden başlat (yalnız |I| < 1 A iken; SAFE'ten çıkış)
 *                c 0x11 1000     EMU: 1 s boyunca CRC'yi boz      (value = süre ms, 0 = sürekli)
 *                c 0x12 1000     EMU: alive counter'ı dondur
 *                c 0x13 300      EMU: 0x101/0x102'yi 300 ms gönderme (kablo kopması taklidi)
 *                c 0x14 0        EMU: NTC kopuk, T = -40 °C (sürekli)
 *                c 0x10 0        EMU: tüm hataları kaldır
 *              Akım sıfırdan geçerken (|I| < 1 A) yeniden başlatma için: "c 0x01 0xA5A5" komutunu
 *              rapordaki akım küçükken gönderin (salınım periyodu 20 s, sıfır geçişi t = 5 s, 15 s).
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
uint8_t       cnt_cmd = 0;                         /* 0x301 alive counter */
uint8_t       cmd_have = 0, cmd_last = 0;          /* hattan gelen 0x301 için tekrar kontrolü */

/* ---------------------------------------------------------------- hata enjeksiyonu (0x10-0x14) */
enum { F_CRC, F_FREEZE, F_SILENT, F_NTC, N_FAULT };
const char *const FAULT_NAME[N_FAULT] = { "CRC", "FREEZE", "SILENT", "NTC" };
uint8_t  f_on[N_FAULT], f_perm[N_FAULT];
uint32_t f_until[N_FAULT];
uint8_t  cnt_frozen = 0;

bool faultActive(uint8_t f, uint32_t now)
{
  if (f_on[f] && !f_perm[f] && (int32_t)(now - f_until[f]) >= 0) f_on[f] = 0;   /* süre doldu */
  return f_on[f];
}

void applyEmuCommand(uint8_t cmd, int16_t value, uint32_t now)
{
  const uint16_t dur = (uint16_t)value;              /* süre [ms], 0 = sürekli */
  if (cmd == CAN_CMD_EMU_CLEAR) {
    memset(f_on, 0, sizeof(f_on));
    Serial.println(F("EMU: all faults cleared"));
    return;
  }
  if (cmd < CAN_CMD_EMU_CRC || cmd > CAN_CMD_EMU_NTC_OPEN) {
    Serial.println(F("EMU: unknown EMU command"));
    return;
  }
  const uint8_t f = cmd - CAN_CMD_EMU_CRC;
  if (f == F_FREEZE && !f_on[F_FREEZE]) cnt_frozen = cnt_tx;
  f_on[f] = 1;  f_perm[f] = (dur == 0);  f_until[f] = now + dur;
  Serial.print(F("EMU: fault "));  Serial.print(FAULT_NAME[f]);
  if (dur) { Serial.print(F(" for ")); Serial.print(dur); Serial.println(F(" ms")); }
  else Serial.println(F(" until cleared (c 0x10 0)"));
}

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
  float T = 25.4f + ((cnt_tx & 1) ? 0.1f : 0.0f);         /* ölçüm gürültüsü taklidi */
  if (faultActive(F_NTC, now)) T = -40.0f;                /* NTC kopuk */
  const can_meas1_t m1 = { V, I, T };
  const can_meas2_t m2 = { V / 19.0f - 0.010f, V / 19.0f + 0.010f, 22.0f };
  const uint8_t c = faultActive(F_FREEZE, now) ? cnt_frozen : cnt_tx;
  const bool bad_crc = faultActive(F_CRC, now);
  uint8_t d[8];

  cnt_tx = (cnt_tx + 1) & 0x0F;                    /* 4 bit alive counter (kayıpta da artar) */
  if (faultActive(F_SILENT, now)) return;          /* hatta hiçbir şey yok */

  can_encode_meas1(&m1, c, d);
  if (bad_crc) d[7] ^= 0xFF;
  bool ok1 = sendRaw(CAN_ID_EMU_MEAS1, d);
  can_encode_meas2(&m2, c, d);
  if (bad_crc) d[7] ^= 0xFF;
  bool ok2 = sendRaw(CAN_ID_EMU_MEAS2, d);

  n_tx_ok  += ok1 + ok2;
  n_tx_err += !ok1 + !ok2;
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
    } else if (id == CAN_ID_PC_CMD) {               /* başka bir düğümden komut */
      can_cmd_t q;
      if (can_decode_cmd(f.data, &q, &c) && !(cmd_have && c == cmd_last)) {
        cmd_have = 1;  cmd_last = c;
        if (q.cmd >= 0x10 && q.cmd <= 0x1F) applyEmuCommand(q.cmd, q.value, now);
      }
    } else {
      n_rx_other++;
    }
  }
}

/* ---------------------------------------------------------------- seri komut: "c <cmd> <value>" */
void sendCommand(uint8_t cmd, int16_t value, uint32_t now)
{
  const can_cmd_t q = { cmd, value };
  uint8_t d[8];
  can_encode_cmd(&q, cnt_cmd, d);
  const bool ok = sendRaw(CAN_ID_PC_CMD, d);
  Serial.print(F("0x301 cmd 0x"));  Serial.print(cmd, HEX);
  Serial.print(F(" value "));       Serial.print(value);
  Serial.print(F(" counter "));     Serial.print(cnt_cmd);
  Serial.println(ok ? F(" sent") : F(" TX FAILED"));
  if (ok) cnt_cmd = (cnt_cmd + 1) & 0x0F;
  if (cmd >= 0x10 && cmd <= 0x1F) applyEmuCommand(cmd, value, now);
}

void serialCommands(uint32_t now)
{
  static char line[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch != 10 && ch != 13) { if (n < sizeof(line) - 1) line[n++] = ch; continue; }   /* LF / CR */
    if (n == 0) continue;
    line[n] = 0;  n = 0;
    char *p = line, *e;
    while (*p == ' ') p++;
    if (*p != 'c' && *p != 'C') { Serial.println(F("usage: c <cmd> <value>   e.g. c 0x13 300")); continue; }
    p++;
    const long cmd = strtol(p, &e, 0);
    if (e == p || cmd < 0 || cmd > 255) { Serial.println(F("bad cmd")); continue; }
    p = e;
    const long val = strtol(p, &e, 0);
    if (e == p || val < -32768L || val > 65535L) { Serial.println(F("bad value")); continue; }
    sendCommand((uint8_t)cmd, (int16_t)(uint16_t)val, now);
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
  for (uint8_t f = 0; f < N_FAULT; f++)
    if (faultActive(f, now)) { Serial.print(F(" !EMU:")); Serial.print(FAULT_NAME[f]); }
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
  Serial.println(F("=== MCP2515 EMU node, 500 kbit/s, normal mode: TX 0x101/0x102, RX 0x201/0x202, commands: c <cmd> <value> ==="));

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
  serialCommands(now);

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
