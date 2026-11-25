<?php
// Sunucu ayarı. config.php olarak kopyalanır (git dışı); tek ayar API_KEY (gateway.py ile aynı).
// Veritabanı SQLite: tek dosya, PHP ilk istekte kendisi oluşturur (MySQL kurulumu, kullanıcı, parola yok).
// Dosya web kökünün DIŞINDA tutulur (~/bitirmeprj_data/bms.sqlite); yazılamazsa bu klasördeki data/
// alt klasörüne düşer (data/.htaccess dışarıdan erişimi kapatır).
const API_KEY = 'DEGISTIR_uzun_rastgele_anahtar';

function db_file(): string {
    $outside = dirname(__DIR__, 2) . '/bitirmeprj_data';
    if ((is_dir($outside) || @mkdir($outside, 0700, true)) && is_writable($outside)) return $outside . '/bms.sqlite';
    $local = __DIR__ . '/data';
    if (!is_dir($local)) @mkdir($local, 0700, true);
    if (!file_exists($local . '/.htaccess')) @file_put_contents($local . '/.htaccess', "Require all denied\n");
    return $local . '/bms.sqlite';
}

// geniş satır kolonları: kolon => SQLite tipi (gateway alan adı api.php'de)
const TELEMETRY_X = ['cause' => 'INTEGER', 'ts' => 'REAL', 'i_chg' => 'REAL', 'psi' => 'REAL', 'i_meas' => 'REAL',
                     'v_pack' => 'REAL', 't_meas' => 'REAL', 'v_min' => 'REAL', 'v_max' => 'REAL',
                     'psi_t' => 'REAL', 'psi_c' => 'REAL', 'psi_tdot' => 'REAL', 'psi_h' => 'REAL',
                     'psi_s' => 'REAL', 'psi_dom' => 'INTEGER', 'tdot' => 'REAL', 'k_r' => 'REAL',
                     'soh_c' => 'REAL', 'soh_n' => 'INTEGER', 'efc' => 'REAL', 'soh_anchor' => 'INTEGER',
                     'rest_s' => 'INTEGER', 'soh_ratio' => 'REAL'];

function db(): PDO {
    static $pdo = null;
    if ($pdo === null) {
        $pdo = new PDO('sqlite:' . db_file(), null, null,
                       [PDO::ATTR_ERRMODE => PDO::ERRMODE_EXCEPTION,
                        PDO::ATTR_DEFAULT_FETCH_MODE => PDO::FETCH_ASSOC]);
        $pdo->exec('PRAGMA journal_mode=WAL; PRAGMA busy_timeout=3000;');
        $pdo->exec(file_get_contents(__DIR__ . '/schema.sql'));     // CREATE TABLE IF NOT EXISTS
        // eski telemetry tablosuna geniş satır (DX) kolonlarını ekle (schema.sql ile aynı liste)
        $have = array_column($pdo->query('PRAGMA table_info(telemetry)')->fetchAll(), 'name');
        foreach (TELEMETRY_X as $c => $t) {
            if (!in_array($c, $have, true)) $pdo->exec("ALTER TABLE telemetry ADD COLUMN $c $t");
        }
    }
    return $pdo;
}

function out($data, int $code = 200): void {
    http_response_code($code);
    header('Content-Type: application/json; charset=utf-8');
    header('Cache-Control: no-store');
    echo json_encode($data);
    exit;
}
