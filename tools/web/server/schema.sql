-- SQLite şeması: config.php her bağlantıda çalıştırır (IF NOT EXISTS), elle çalıştırmak gerekmez.
-- Zamanlar UTC (CURRENT_TIMESTAMP). mock_server.py aynı şemayı kullanır.
CREATE TABLE IF NOT EXISTS runs (
  run_id     INTEGER PRIMARY KEY,
  name       TEXT NOT NULL,
  started_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
  ended_at   TEXT,
  result     TEXT NOT NULL DEFAULT 'RUNNING',        -- RUNNING | PASS | FAIL
  summary    TEXT
);

CREATE TABLE IF NOT EXISTS telemetry (
  id       INTEGER PRIMARY KEY AUTOINCREMENT,
  run_id   INTEGER NOT NULL DEFAULT 0,               -- 0 = canlı mod, > 0 = Monte Carlo koşusu
  t_ms     INTEGER NOT NULL,
  soc      REAL, soh REAL, temp REAL, i_allow REAL,
  state    INTEGER,                                  -- 1 NORMAL, 2 DERATE, 3 LIMP-HOME, 4 SAFE-STOP (bms_algo.h)
  tec      INTEGER, rec INTEGER,
  rx_at    TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
  -- geniş satır (DX, BMS_TELEMETRY=2); yoksa NULL. Eski tabloya config.php db() ALTER ile ekler.
  cause    INTEGER,                                  -- neden bitleri (bms_algo.h)
  ts REAL, i_chg REAL, psi REAL,                     -- yüzey kestirimi, şarj I_allow, toplam kısıtlama 1 - I_allow/I_nom
  i_meas REAL, v_pack REAL, t_meas REAL,             -- 0x101 ölçümü: akım (+ deşarj), paket gerilimi, sıcaklık
  v_min REAL, v_max REAL,                            -- 0x102: en düşük / en yüksek grup gerilimi
  psi_t REAL, psi_c REAL, psi_tdot REAL, psi_h REAL, -- stres alt indeksleri: çekirdek T, soğutma, dT/dt, direnç (k_R)
  psi_s REAL, psi_dom INTEGER,                       -- çok kriterli stres indeksi Ψ, baskın kriter (0 yok, 1 T, 2 C, 3 Tdot, 4 H)
  tdot REAL, k_r REAL,                               -- süzülmüş yüzey dT/dt (°C/dk), EKF R0 ölçeği
  soh_c REAL, soh_n INTEGER, efc REAL,               -- kapasite SOH'u: öğrenilen %, ölçüm sayısı, eşdeğer tam çevrim
  soh_anchor INTEGER, rest_s INTEGER, soh_ratio REAL, -- çapa var mı, dinlenme süresi [s], son ölçüm oranı %
  psi_a REAL, psi_b REAL, psi_p REAL, -- yaşlanma hızı, dengesizlik, deşarjı kısan koruma payı Ψ_P
  r_cyc REAL, r_cal REAL, dmg REAL, -- yaşlanma hızı oranları (çevrim, takvim; 1 = 25 °C referansı), birikimli hasar
  v_kmh REAL -- araç hızı (gateway, twin_emu R satırından; ECU telemetrisinde yok)
);
CREATE INDEX IF NOT EXISTS telemetry_run ON telemetry (run_id, t_ms);

CREATE TABLE IF NOT EXISTS events (
  id     INTEGER PRIMARY KEY AUTOINCREMENT,
  run_id INTEGER NOT NULL DEFAULT 0,
  t_ms   INTEGER NOT NULL,
  code   TEXT NOT NULL,
  detail TEXT,
  rx_at  TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
);
CREATE INDEX IF NOT EXISTS events_run ON events (run_id);

-- Bağlantı kalitesi: POST yığını başına bir satır (gateway.py meta'sı; meta'sız istemcide yalnız sayılar).
-- *_ms alanları Unix zamanı (ms). srv_ms sunucu saati, diğerleri gateway (PC) saati: tek yön gecikme
-- (srv_ms - sent_ms) iki saatin NTP ile eş olmasına bağlıdır; rtt/age tek saatle ölçülür, onlar esas.
CREATE TABLE IF NOT EXISTS link (
  id           INTEGER PRIMARY KEY AUTOINCREMENT,
  seq          INTEGER UNIQUE,                       -- gateway yığın sırası (artan); NULL = meta'sız istemci
  srv_ms       INTEGER NOT NULL,
  sent_ms      INTEGER, rx_first_ms INTEGER, rx_last_ms INTEGER,
  n_rows       INTEGER NOT NULL, n_events INTEGER NOT NULL,
  uart_missing INTEGER, uart_bad INTEGER, jit_ms INTEGER,
  prev_rtt_ms  INTEGER, prev_age_ms INTEGER,        -- önceki yığının gateway'de ölçülen RTT ve uçtan uca gecikmesi
  resent       INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS link_srv ON link (srv_ms);
