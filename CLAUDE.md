# bms_ecu

BMS ECU firmware: 19S12P LG INR21700 M50T paket. Hedef **NUCLEO-G431RB (STM32G431RBTx, Cortex-M4F, 170 MHz)**.
STM32CubeIDE projesi, **HAL yok**: çevre birimlerine doğrudan register seviyesinde (CMSIS) erişilir.

## Yapı
- `Src/main.c`: `APP_MODE` ile uygulama seçimi (`APP_PIL`=1, `APP_CAN_LOOPBACK`=2, `APP_CAN_NODE`=3, `APP_ECU`=4 varsayılan, `APP_CAN_SOAK`=5, `APP_CORDIC`=6, `APP_LA`=7); `-DAPP_MODE=N` ile de verilir. Her uygulama geri dönmez.
- `APP_ECU` derleme bayrakları (varsayılan 0, CubeIDE derlemesi ve PIL bunlardan etkilenmez): `BMS_SAFETY` (IWDG ~100 ms, açılışta flash CRC, reset nedeni; watchdog reseti / CRC hatası → kalıcı SAFE; komut 0x04 = sonsuz döngü testi), `BMS_OUTPUTS` (far PWM PA8/PA9 TIM1 1 kHz, kontaktör PB5), `BMS_TELEMETRY` (her adımda `D,...` satırı → `tools/live_panel.py`).
- `bms/bsp/`: donanım katmanı. `bsp.*` = FPU, saat, LED (PA5), LPUART1 115200, DWT döngü sayacı. `bsp_can.*` = FDCAN1 klasik CAN 500 kbit/s, polling, bus-off kurtarma. `bsp_safety.*` IWDG/CRC/reset nedeni, `bsp_out.*` far PWM + kontaktör, `bsp_cordic.*` CORDIC expf/logf.
- Pinler: PA5 LED, PA2/PA3 LPUART1, PA11/PA12 FDCAN1, PA8 kısa far (D7, ULN IN1), PA9 uzun far (D8, ULN IN2), PB5 kontaktör (D4, düşükte tetiklenen 5 V röle modülü IN; açık-akaçlı, `BSP_OUT_K_OPEN_DRAIN`=1 varsayılan). Lojik analizör TIM6 + DMA1 kanal 1 (DMAMUX istek 8) kullanır.
- `bms/common/`: **platformdan bağımsız** algoritmalar (STM32 + PC SIL testinde ortak). Buraya donanım/bsp bağımlılığı EKLEME.
  - `bms_config.h`: tüm sabitler. Değerler `simulink/bms_param_struct.m` ve `twin/*.py` ile birebir aynı olmalı; biri değişirse diğerleri de değişmeli (kullanıcıya hatırlat).
  - `bms_algo.*`: SOC EKF `[SOC V1 V2 b_I]`, sıcaklık gözlemcisi `[Tc Ts g]`, sensör teşhisi, çevrimiçi direnç kestiricisi, karar mekanizması (NORMAL/DERATE/LIMP/SAFE). Giriş noktası `bms_init` / `bms_step` (Ts = 0.1 s).
  - `bms_can.*`: CAN mesaj kodlama, CRC-8 SAE J1850. ID'ler: 0x101/0x102 ölçüm (EMU→ECU), 0x201 kontrol, 0x202 durum (ECU→), 0x301 PC komutu.
    0x301 komut kodları `bms_can.h`'de (`CAN_CMD_*`): 0x01-0x0F ECU (korumalı yeniden başlatma, SOC ayarla, sayaç sil), 0x10-0x1F EMU hata enjeksiyonu (CRC, donmuş sayaç, sessizlik, NTC kopuk). PC'den gönderim: `mcp2515_node` Serial Monitor'üne `c <cmd> <value>`.
  - `soh_tree.*`: üretilmiş ML karar ağacı (`ml/soh_tree.c` kaynaklı); elle düzenleme.
- `bms/pil/`: test uygulamaları. `app_pil.c` UART ikili protokol (PC tarafı `pil_test.py`), `app_can_lb.c` loopback testi, `app_can_node.c` gerçek hat (yalnız CAN), `app_ecu.c` tam zincir (`APP_ECU`=4): CAN 0x101/0x102 → `bms_step` → 0x201/0x202.
- `arduino/`: Arduino Uno + MCP2515 (8 MHz, CS D10, INT D2) karşı düğüm sketch'leri; CubeIDE derlemesine girmez. `mcp2515_loopback/` iç geri döngü testi, `mcp2515_node/` gerçek hatta EMU düğümü (0x101/0x102 gönderir, 0x202 alır).
  - `mcp2515_bridge/` PC ↔ CAN köprüsü: `tools/pil_test/can_replay.py` kayıtlı veriyi (`tools/pil_test/data/*_in.csv`) CAN'dan oynatır, ECU'nun 0x201/0x202 cevabını SIL çıktısıyla karşılaştırır. Seri protokol sketch'in başında; değişirse `can_replay.py` de güncellenmeli.
  - `arduino/mcp2515_node/bms_can.*` ve `arduino/mcp2515_bridge/bms_can.*` = `bms/common/bms_can.*`'nin **birebir kopyası**. `bms_can.*` değişirse kopyaları da güncelle.
  - `app_ecu.c` adımı ölçüm çifti (0x101 + 0x102) gelince atar; oynatma testinin örnek örnek karşılaştırması buna dayanır.
- `arduino/can_soak/`: 10 dk CAN dayanıklılık testinin göndereni (0x101, 16 bit sayaç, 10 Hz).
- `tools/`: `fw_build.py` (CubeIDE'den bağımsız derleme + ST-LINK yükleme, `--crc-ref`), `can_soak_run.py`, `la_view.py` (lojik analizör çözücüsü, PNG), `live_panel.py` (canlı panel, CSV), `out_demo.py` (far/kontaktör yavaş gösterimi, `BMS_OUTPUTS=1` ile). Çıktılar `build/` (git dışı).
- `tools/pil_test/`: PC tarafı test betikleri (Python; CubeIDE derlemesine girmez). `pil_test.py` UART PIL, `can_replay.py` CAN oynatma. `data/<senaryo>_in.csv` girdi, `data/<senaryo>_c.csv` PC SIL referans çıktısı (`C:\bitirme`'den üretilir). Betiklerin yazdığı sonuç CSV'leri git dışı.
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
- STM32 test yazılımlarını Claude derleyip yükler: `python tools/fw_build.py --app N [--opt O0] [--define X=1] --flash` (kullanıcı onayı, 2026-11-15). Test bitince varsayılanı geri yükle: `--app 4 --flash`. Arduino sketch'lerini kullanıcı Arduino IDE'de yükler (Dosya > Aç ile, sketch klasöründen).
  Makefile'lar (`Debug/*.mk`, `objects.list`) CubeIDE'nin ürettiği dosyalar; komut satırından `make` yeni kaynak dosyalarını görmez, bu yüzden derleme için kullanma.
- ECU testlerinde önce EMU (COM10'u açmak Uno'yu sıfırlar, ~2 s susar), sonra ECU başlatılmalı; dayanıklılık testinde tersi (alıcı önce).
- STM32_Programmer_CLI kurulu: `C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_*\tools\bin\STM32_Programmer_CLI.exe`.
  Sıfırlama (`-c port=SWD mode=HOTPLUG -rst`), hedef gerilimi ve register okuma için kullanılabilir (`can_replay.py` de sıfırlamayı bununla yapar). CubeIDE'de hata ayıklama oturumu açıkken ST-LINK'e bağlanamaz.
- Seri portlar: COM9 = STM32 ST-LINK VCP (LPUART1), COM10 = Arduino Uno (CH340). Porttan okumadan önce Arduino IDE'deki Serial Monitor kapatılmalı.
- Git: `main` dalı. `git` PATH'te olmayabilir; `C:\Program Files\Git\cmd\git.exe` kullan.
