# bms_ecu

BMS ECU firmware: 19S12P LG INR21700 M50T paket. Hedef **NUCLEO-G431RB (STM32G431RBTx, Cortex-M4F, 170 MHz)**.
STM32CubeIDE projesi, **HAL yok**: çevre birimlerine doğrudan register seviyesinde (CMSIS) erişilir.

## Yapı
- `Src/main.c`: `APP_MODE` ile uygulama seçimi (`APP_PIL`=1, `APP_CAN_LOOPBACK`=2, `APP_CAN_NODE`=3, `APP_ECU`=4 varsayılan). Her uygulama geri dönmez.
- `bms/bsp/`: donanım katmanı. `bsp.*` = FPU, saat, LED (PA5), LPUART1 115200, DWT döngü sayacı. `bsp_can.*` = FDCAN1 klasik CAN 500 kbit/s, polling, bus-off kurtarma.
- `bms/common/`: **platformdan bağımsız** algoritmalar (STM32 + PC SIL testinde ortak). Buraya donanım/bsp bağımlılığı EKLEME.
  - `bms_config.h`: tüm sabitler. Değerler `simulink/bms_param_struct.m` ve `twin/*.py` ile birebir aynı olmalı; biri değişirse diğerleri de değişmeli (kullanıcıya hatırlat).
  - `bms_algo.*`: SOC EKF `[SOC V1 V2 b_I]`, sıcaklık gözlemcisi `[Tc Ts g]`, sensör teşhisi, çevrimiçi direnç kestiricisi, karar mekanizması (NORMAL/DERATE/LIMP/SAFE). Giriş noktası `bms_init` / `bms_step` (Ts = 0.1 s).
  - `bms_can.*`: CAN mesaj kodlama, CRC-8 SAE J1850. ID'ler: 0x101/0x102 ölçüm (EMU→ECU), 0x201 kontrol, 0x202 durum (ECU→), 0x301 PC komutu.
  - `soh_tree.*`: üretilmiş ML karar ağacı (`ml/soh_tree.c` kaynaklı); elle düzenleme.
- `bms/pil/`: test uygulamaları. `app_pil.c` UART ikili protokol (PC tarafı `pil_test.py`), `app_can_lb.c` loopback testi, `app_can_node.c` gerçek hat (yalnız CAN), `app_ecu.c` tam zincir (`APP_ECU`=4): CAN 0x101/0x102 → `bms_step` → 0x201/0x202.
- `arduino/`: Arduino Uno + MCP2515 (8 MHz, CS D10, INT D2) karşı düğüm sketch'leri; CubeIDE derlemesine girmez. `mcp2515_loopback/` iç geri döngü testi, `mcp2515_node/` gerçek hatta EMU düğümü (0x101/0x102 gönderir, 0x202 alır).
  - `mcp2515_bridge/` PC ↔ CAN köprüsü: `pil_test/can_replay.py` kayıtlı veriyi (`pil_test/data/*_in.csv`) CAN'dan oynatır, ECU'nun 0x201/0x202 cevabını SIL çıktısıyla karşılaştırır. Seri protokol sketch'in başında; değişirse `can_replay.py` de güncellenmeli.
  - `arduino/mcp2515_node/bms_can.*` ve `arduino/mcp2515_bridge/bms_can.*` = `bms/common/bms_can.*`'nin **birebir kopyası**. `bms_can.*` değişirse kopyaları da güncelle.
  - `app_ecu.c` adımı ölçüm çifti (0x101 + 0x102) gelince atar; oynatma testinin örnek örnek karşılaştırması buna dayanır.
- Donanım: FDCAN1 PA11 (RX) / PA12 (TX), AF9 → TJA1050 transceiver (5 V beslemeli). PA11/PA12 FT_u; RXD 5 V sürdüğü için bu pinlerde iç pull-up/pull-down AÇMA (DS12589).
- `Drivers/` (CMSIS), `Startup/`, `*.ld`: üretici dosyaları, dokunma.

## Kodlama kuralları
- C, float32 (`f` sonekli sabitler; `double` kullanma — FPU yalnız tek hassasiyet).
- Dinamik bellek yok, `printf` yok. Modül deseni: durum `struct` + `*_init` + `*_step`.
- Yorumlar ve açıklamalar **Türkçe**; mevcut stili (kısa blok yorum başlıkları) koru.
- Protokol/çerçeve düzeni (bayt ofsetleri, CRC kapsamı) değişirse dosya başındaki yorumu ve PC tarafı scripti de güncellenmeli.
- Zamanlama kritik: `bms_step` süresi DWT ile ölçülüyor (`app_pil.c`); döngü içine ağır iş ekleme.

## Derleme
- STM32CubeIDE, Debug (-O2) / Release (-Os). Include yolları: `Inc`, `bms/bsp`, `bms/common`, `bms/pil`, `Drivers/CMSIS/*`.
- Komut satırından derleme/flash bu ortamda kurulu değil; değişiklikten sonra kullanıcıdan CubeIDE'de derlemesini iste.
- Git: `main` dalı. `git` PATH'te olmayabilir; `C:\Program Files\Git\cmd\git.exe` kullan.
