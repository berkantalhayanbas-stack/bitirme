<?php
// GET ?q=live&since=<id>   → son telemetri (en fazla 600 satır)
// GET ?q=runs              → senaryo listesi
// GET ?q=run&id=<run_id>   → tek senaryonun tüm verisi + olaylar
// GET ?q=ping              → sunucu kontrolü (SQLite sürümü, satır sayısı)
require __DIR__ . '/config.php';
$pdo = db();
$q = $_GET['q'] ?? 'live';

if ($q === 'live') {
    $since = (int)($_GET['since'] ?? 0);
    if ($since === 0) {
        $rows = $pdo->query('SELECT * FROM (SELECT * FROM telemetry ORDER BY id DESC LIMIT 600) x ORDER BY id')->fetchAll();
    } else {
        $st = $pdo->prepare('SELECT * FROM telemetry WHERE id > ? ORDER BY id LIMIT 600');
        $st->execute([$since]);
        $rows = $st->fetchAll();
    }
    $ev = $pdo->query('SELECT * FROM events ORDER BY id DESC LIMIT 20')->fetchAll();
    out(['rows' => $rows, 'events' => $ev]);
}

if ($q === 'runs') {
    out($pdo->query("SELECT r.*, CAST(strftime('%s', ended_at) - strftime('%s', started_at) AS INTEGER) AS dur_s
                     FROM runs r ORDER BY started_at DESC, run_id DESC LIMIT 200")->fetchAll());
}

if ($q === 'run') {
    $id = (int)($_GET['id'] ?? 0);
    $st = $pdo->prepare('SELECT t_ms,soc,soh,temp,i_allow,state FROM telemetry WHERE run_id=? ORDER BY t_ms');
    $st->execute([$id]);
    $se = $pdo->prepare('SELECT t_ms,code,detail FROM events WHERE run_id=? ORDER BY t_ms');
    $se->execute([$id]);
    out(['rows' => $st->fetchAll(), 'events' => $se->fetchAll()]);
}

if ($q === 'ping') {
    out(['ok' => true, 'sqlite' => $pdo->query('SELECT sqlite_version()')->fetchColumn(),
         'telemetry' => (int)$pdo->query('SELECT COUNT(*) FROM telemetry')->fetchColumn(),
         'outside_webroot' => strpos(db_file(), __DIR__) !== 0]);
}

out(['error' => 'bad q'], 400);
