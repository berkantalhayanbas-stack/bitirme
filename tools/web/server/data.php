<?php
// GET ?q=live&since=<id>&n=<k> → son telemetri (since=0: son k satır, en fazla 3000; sonra en fazla 600)
// GET ?q=runs              → senaryo listesi
// GET ?q=run&id=<run_id>   → tek senaryonun tüm verisi + olaylar
// GET ?q=link&min=5        → son N dakikanın bağlantı kalitesi (p50/p95, kayıp)
// GET ?q=linkhist&min=5    → son N dakikanın yığın başına RTT / uçtan uca gecikme / kayıp (grafik)
// GET ?q=ping              → sunucu kontrolü (SQLite sürümü, satır sayısı)
require __DIR__ . '/config.php';

// yüzdelik (en yakın sıra); boşsa null. mock_server.py'deki pct() ile aynı
function pct(array $v, float $q) {
    $v = array_values(array_filter($v, function ($x) { return $x !== null; }));
    if (!$v) return null;
    sort($v);
    return (int)$v[min(count($v) - 1, (int)round($q / 100 * (count($v) - 1)))];
}
$pdo = db();
$q = $_GET['q'] ?? 'live';

if ($q === 'live') {
    $since = (int)($_GET['since'] ?? 0);
    if ($since === 0) {
        $k = max(1, min(3000, (int)($_GET['n'] ?? 600)));
        $rows = $pdo->query("SELECT * FROM (SELECT * FROM telemetry ORDER BY id DESC LIMIT $k) x ORDER BY id")->fetchAll();
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
    $st = $pdo->prepare('SELECT t_ms,soc,soh,temp,i_allow,state,cause,ts,i_chg,i_meas,v_min,v_max
                         FROM telemetry WHERE run_id=? ORDER BY t_ms');
    $st->execute([$id]);
    $se = $pdo->prepare('SELECT t_ms,code,detail FROM events WHERE run_id=? ORDER BY t_ms');
    $se->execute([$id]);
    out(['rows' => $st->fetchAll(), 'events' => $se->fetchAll()]);
}

if ($q === 'link') {
    $min = max(1, min(1440, (int)($_GET['min'] ?? 5)));
    $st = $pdo->prepare('SELECT * FROM link WHERE srv_ms >= ? ORDER BY id');
    $st->execute([(int)round(microtime(true) * 1000) - $min * 60000]);
    $L = $st->fetchAll();
    $rows = array_sum(array_column($L, 'n_rows'));
    $miss = array_sum(array_column($L, 'uart_missing'));
    $col = function ($k) use ($L) { return array_column($L, $k); };
    $oneway = array_map(function ($r) { return $r['sent_ms'] === null ? null : $r['srv_ms'] - $r['sent_ms']; }, $L);
    out(['minutes' => $min, 'batches' => count($L), 'rows' => $rows, 'uart_missing' => $miss,
         'uart_bad' => array_sum($col('uart_bad')), 'resent' => array_sum($col('resent')),
         'loss_pct' => $rows + $miss ? round(100 * $miss / ($rows + $miss), 3) : null,
         'rtt_p50' => pct($col('prev_rtt_ms'), 50), 'rtt_p95' => pct($col('prev_rtt_ms'), 95),
         'age_p50' => pct($col('prev_age_ms'), 50), 'age_p95' => pct($col('prev_age_ms'), 95),
         'jit_p95' => pct($col('jit_ms'), 95), 'oneway_p50' => pct($oneway, 50),
         'last_srv_ms' => $L ? (int)end($L)['srv_ms'] : null]);
}

if ($q === 'linkhist') {
    $min = max(1, min(1440, (int)($_GET['min'] ?? 5)));
    $st = $pdo->prepare('SELECT srv_ms,prev_rtt_ms,prev_age_ms,n_rows,uart_missing,uart_bad,resent
                         FROM link WHERE srv_ms >= ? ORDER BY id LIMIT 2000');
    $st->execute([(int)round(microtime(true) * 1000) - $min * 60000]);
    out(['now_ms' => (int)round(microtime(true) * 1000), 'rows' => $st->fetchAll()]);
}

if ($q === 'ping') {
    out(['ok' => true, 'sqlite' => $pdo->query('SELECT sqlite_version()')->fetchColumn(),
         'telemetry' => (int)$pdo->query('SELECT COUNT(*) FROM telemetry')->fetchColumn(),
         'outside_webroot' => strpos(db_file(), __DIR__) !== 0]);
}

out(['error' => 'bad q'], 400);
