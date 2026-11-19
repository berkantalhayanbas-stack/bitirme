# İlerleme günlüğü

Otonom çalışma oturumlarının kaydı (en yeni en üstte). Her madde bir commit'e karşılık gelir.
Kural: PC'de doğrulanamayan hiçbir şey "doğrulandı" diye yazılmaz; kart/Arduino testi gerekenler
"kartta denenmedi" diye işaretlenir.

## 2026-11-19 gece oturumu (dal `ccr-1a55f758-sxw6pi`)

Kısıt: bulut ortamında NUCLEO / ST-LINK / Arduino yok; yalnız PC'de gcc ile doğrulanabilen işler yapıldı.
`bms/common` algoritma çıktıları değişmedi (`sil_ref.py --check`: S3 fark 0, S6 en çok 0,00003 puan SOC).

- [x] **PC birim testleri** `tools/host_tests/` (`python tools/host_tests/run.py`, ASan+UBSan):
      CRC/kodlama/çözme, hücre çerçeveleri, izleyici zaman aşımları, dengeleme, karar mekanizması,
      bms_step sağlamlığı. 121 kontrol, hepsi geçiyor.
- [x] **Hata düzeltmesi (savunmacı):** `decision_step` NaN/Inf girdide hiçbir sert sınıra
      takılmıyordu (NaN karşılaştırmaları hep yanlış). Geçerli CRC'li ama saçma ölçümlerde (örn. V = 0)
      EKF 66. adımda NaN üretiyordu. Artık sonlu olmayan girdi -> SAFE (neden 128), `bms_step` çıktıları
      0'a sabitlenir (0x202'ye NaN gitmesin). Normal çalışmada etkisi yok. Kartta denenmedi.
