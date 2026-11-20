<?php
// config.sample.php -> config.php olarak kopyala, cPanel > MySQL Databases bilgilerini gir (config.php git dışı)
const DB_HOST = 'localhost';
const DB_NAME = 'KULLANICI_bitirme';
const DB_USER = 'KULLANICI_bms';
const DB_PASS = 'DEGISTIR';
const API_KEY = 'DEGISTIR_uzun_rastgele_anahtar';   // gateway.py ile aynı

function db(): PDO {
    static $pdo = null;
    if ($pdo === null) {
        $pdo = new PDO('mysql:host=' . DB_HOST . ';dbname=' . DB_NAME . ';charset=utf8mb4',
                       DB_USER, DB_PASS,
                       [PDO::ATTR_ERRMODE => PDO::ERRMODE_EXCEPTION,
                        PDO::ATTR_DEFAULT_FETCH_MODE => PDO::FETCH_ASSOC]);
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
