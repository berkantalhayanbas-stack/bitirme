/* mcp2515_loopback.ino -- Arduino Uno + MCP2515 modülü, iç geri döngü (loopback) testi.
 *
 * Kütüphane : "mcp2515" (autowp), Library Manager'dan
 * Bağlantı  : CS = D10, INT = D2, SPI: D11 MOSI, D12 MISO, D13 SCK, VCC 5 V, GND
 * CAN       : 500 kbit/s, modül kristali 8 MHz (kristalin üstünde "8.000" yazmalı)
 *
 * Loopback modunda MCP2515 gönderdiği çerçeveyi kendi alıcısına içeriden döndürür; CAN hattına
 * (transceiver, CANH/CANL) hiçbir şey çıkmaz. Kablo, sonlandırma ve karşı düğüm gerekmez.
 *
 * Her 500 ms: 0x101 kimlikli 8 baytlık çerçeve gönder -> INT (D2) düşük olana kadar bekle ->
 * çerçeveyi oku -> gönderileni ve alınanı Serial'a (115200) yaz -> eşleşirse "OK".
 */
#include <SPI.h>
#include <mcp2515.h>

const uint8_t  PIN_CS      = 10;
const uint8_t  PIN_INT     = 2;
const uint16_t TX_ID       = 0x101;
const uint32_t PERIOD_MS   = 500;
const uint32_t RX_TIMEOUT  = 50;     /* ms; 8 baytlık çerçeve 500 kbit/s'de ~0,25 ms */

MCP2515 mcp2515(PIN_CS);

uint8_t  counter = 0;
uint32_t n_ok = 0, n_fail = 0;
uint32_t t_next = 0;

/* ---------------------------------------------------------------- yardımcılar */
void printFrame(const char *label, const struct can_frame &f)
{
  char buf[8];
  Serial.print(label);
  Serial.print(" id=0x");
  sprintf(buf, "%03lX", (unsigned long)(f.can_id & CAN_SFF_MASK));
  Serial.print(buf);
  Serial.print(" dlc=");
  Serial.print(f.can_dlc);
  Serial.print(" data=");
  for (uint8_t i = 0; i < f.can_dlc && i < 8; i++) {
    sprintf(buf, "%02X ", f.data[i]);
    Serial.print(buf);
  }
  Serial.println();
}

bool sameFrame(const struct can_frame &a, const struct can_frame &b)
{
  if ((a.can_id & CAN_SFF_MASK) != (b.can_id & CAN_SFF_MASK)) return false;
  if ((a.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != (b.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG))) return false;
  if (a.can_dlc != b.can_dlc) return false;
  return memcmp(a.data, b.data, a.can_dlc) == 0;
}

void fail(const char *why)
{
  n_fail++;
  Serial.print("FAIL: ");
  Serial.println(why);
}

/* ---------------------------------------------------------------- kurulum */
void setup()
{
  Serial.begin(115200);
  while (!Serial) { }
  pinMode(PIN_INT, INPUT_PULLUP);     /* INT aktif düşük: düşük = bekleyen kesme var. Pull-up,
                                         modülde direnç yoksa pinin boşta kalmasını önler */

  Serial.println();
  Serial.println(F("=== MCP2515 loopback test, 500 kbit/s, 8 MHz ==="));

  mcp2515.reset();                    /* RX0/RX1 ve hata kesmeleri açılır, maske/filtre: hepsini kabul */
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println(F("setBitrate FAILED (SPI baglantisi / CS pini?)"));
    while (1) { }
  }
  if (mcp2515.setLoopbackMode() != MCP2515::ERROR_OK) {
    Serial.println(F("setLoopbackMode FAILED (SPI baglantisi?)"));
    while (1) { }
  }
  Serial.println(F("MCP2515 loopback mode OK"));
  t_next = millis();
}

/* ---------------------------------------------------------------- döngü */
void loop()
{
  if ((int32_t)(millis() - t_next) < 0) return;
  t_next += PERIOD_MS;

  struct can_frame tx, rx;
  tx.can_id  = TX_ID;                 /* standart 11 bit, veri çerçevesi */
  tx.can_dlc = 8;
  for (uint8_t i = 0; i < 8; i++) tx.data[i] = (uint8_t)(counter + i * 0x11);
  counter++;

  Serial.print('#');
  Serial.println(counter);
  printFrame("  TX", tx);

  if (mcp2515.sendMessage(&tx) != MCP2515::ERROR_OK) {
    fail("sendMessage (tx buffers full?)");
    return;
  }

  /* INT pini düşük olana kadar bekle (alım kesmesi) */
  const uint32_t t0 = millis();
  while (digitalRead(PIN_INT) == HIGH) {
    if (millis() - t0 > RX_TIMEOUT) {
      fail("timeout, INT (D2) never went low");
      return;
    }
  }

  const MCP2515::ERROR r = mcp2515.readMessage(&rx);
  if (r != MCP2515::ERROR_OK) {
    /* INT düşük ama mesaj yok: hata kesmesi olabilir; bayrakları göster ve temizle */
    Serial.print(F("  CANINTF=0x"));
    Serial.println(mcp2515.getInterrupts(), HEX);
    mcp2515.clearInterrupts();
    fail("INT low but no message");
    return;
  }
  printFrame("  RX", rx);

  if (sameFrame(tx, rx)) {
    n_ok++;
    Serial.print(F("  OK"));
  } else {
    n_fail++;
    Serial.print(F("  FAIL: RX != TX"));
  }
  Serial.print(F("  (ok "));
  Serial.print(n_ok);
  Serial.print(F(", fail "));
  Serial.print(n_fail);
  Serial.println(F(")"));
}
