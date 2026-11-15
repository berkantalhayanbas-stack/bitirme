/* can_soak.ino -- Arduino Uno + MCP2515, 10 dakikalık CAN dayanıklılık testinin gönderen tarafı.
 *
 * Karşı düğüm: STM32 APP_MODE = APP_CAN_SOAK (bms/pil/app_can_soak.c); sonucu o verir.
 * Kütüphane : "mcp2515" (autowp).  Bağlantı: CS = D10, SPI D11/D12/D13, 8 MHz kristal, 500 kbit/s
 *
 * Her 100 ms (10 Hz, micros() ile, kaymasız): ID 0x101, DLC 8, bayt 0-1 = artan 16 bit sayaç
 * (little-endian), diğer baytlar 0. Gönderim başarısızsa (Tx tamponları dolu) sayaç YİNE artar,
 * böylece STM32 bunu kayıp olarak görür.
 * Her 10 s: Serial'a (115200) geçen süre, gönderilen, gönderilemeyen, TEC, REC, EFLG.
 */
#include <SPI.h>
#include <mcp2515.h>

const uint8_t  PIN_CS    = 10;
const uint32_t PERIOD_US = 100000UL;   /* 10 Hz */
const uint32_t REPORT_MS = 10000UL;

MCP2515 mcp2515(PIN_CS);
uint16_t seq = 0;
uint32_t n_ok = 0, n_err = 0, t_next = 0, t_rep = 0, t0 = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("=== CAN soak sender: 0x101, 16-bit counter, 10 Hz, 500 kbit/s ==="));
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println(F("setBitrate FAILED (SPI / CS?)"));
    while (1) { }
  }
  mcp2515.setNormalMode();
  t0 = millis();
  t_rep = t0 + REPORT_MS;
  t_next = micros();
}

void loop()
{
  struct can_frame f;
  while (mcp2515.readMessage(&f) == MCP2515::ERROR_OK) { }   /* gelen her şeyi at */

  if ((int32_t)(micros() - t_next) >= 0) {
    t_next += PERIOD_US;
    f.can_id  = 0x101;
    f.can_dlc = 8;
    memset(f.data, 0, 8);
    f.data[0] = (uint8_t)(seq & 0xFF);
    f.data[1] = (uint8_t)(seq >> 8);
    if (mcp2515.sendMessage(&f) == MCP2515::ERROR_OK) n_ok++; else n_err++;
    seq++;
  }

  const uint32_t now = millis();
  if ((int32_t)(now - t_rep) >= 0) {
    t_rep += REPORT_MS;
    Serial.print(F("t="));        Serial.print((now - t0) / 1000);
    Serial.print(F("s sent "));   Serial.print(n_ok);
    Serial.print(F(" failed "));  Serial.print(n_err);
    Serial.print(F(" | TEC "));   Serial.print(mcp2515.errorCountTX());
    Serial.print(F(" REC "));     Serial.print(mcp2515.errorCountRX());
    Serial.print(F(" EFLG 0x"));  Serial.println(mcp2515.getErrorFlags(), HEX);
  }
}
