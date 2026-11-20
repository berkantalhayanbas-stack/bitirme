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
  rx_at    TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
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
