<?php
// POST JSON: {"rows":[...], "events":[...], "runs":[...], "meta":{...}}  Header: X-API-Key  (SQLite, config.php)
// rows: run,t,soc,soh,T,I,st,tec,rec + isteğe bağlı geniş alanlar (cause,ts,i_chg,psi,i_meas,v_pack,t_meas,v_min,v_max)
// meta (isteğe bağlı, gateway.py): seq, sent_ms, rx_first, rx_last, uart_missing, uart_bad, jit_ms,
// prev_rtt_ms, prev_age_ms, resent. seq <= en son kabul edilen seq ise tekrar oynatma: 409, hiçbir şey yazılmaz.
// meta'sız istemci (esp_gateway) kabul edilir; link tablosuna yalnız sayılar yazılır.
require __DIR__ . '/config.php';

if ($_SERVER['REQUEST_METHOD'] !== 'POST') out(['error' => 'POST only'], 405);
if (!hash_equals(API_KEY, $_SERVER['HTTP_X_API_KEY'] ?? '')) out(['error' => 'unauthorized'], 401);

$in = json_decode(file_get_contents('php://input'), true);
if (!is_array($in)) out(['error' => 'bad json'], 400);
$meta = is_array($in['meta'] ?? null) ? $in['meta'] : null;
$iv = function ($k) use ($meta) { return isset($meta[$k]) && is_numeric($meta[$k]) ? (int)$meta[$k] : null; };
$seq = $iv('seq');

$pdo = db();
$pdo->beginTransaction();
try {
    if ($seq !== null) {
        $last = $pdo->query('SELECT MAX(seq) FROM link')->fetchColumn();
        if ($last !== null && $last !== false && $seq <= (int)$last) {
            $pdo->rollBack();
            out(['error' => 'replay', 'last_seq' => (int)$last], 409);
        }
    }
    $n = 0;
    // önce başlayan senaryolar: aynı numara yeniden koşuyorsa eski izi sil (canlı mod run 0 korunur)
    $rs = $pdo->prepare("INSERT INTO runs (run_id,name) VALUES (?,?)
                         ON CONFLICT(run_id) DO UPDATE SET name=excluded.name, started_at=CURRENT_TIMESTAMP,
                         ended_at=NULL, result='RUNNING', summary=NULL");
    $dt = $pdo->prepare('DELETE FROM telemetry WHERE run_id=? AND run_id>0');
    $de = $pdo->prepare('DELETE FROM events WHERE run_id=? AND run_id>0');
    foreach ($in['runs'] ?? [] as $r) {
        if ($r['op'] !== 'start') continue;
        $dt->execute([(int)$r['id']]);  $de->execute([(int)$r['id']]);
        $rs->execute([(int)$r['id'], substr($r['name'], 0, 64)]);
    }

    // geniş satır alanları (DX) isteğe bağlı: JSON anahtarı = kolon adı (config.php TELEMETRY_X), yoksa NULL
    $xc = array_keys(TELEMETRY_X);
    $st = $pdo->prepare('INSERT INTO telemetry (run_id,t_ms,soc,soh,temp,i_allow,state,tec,rec,' . implode(',', $xc) . ')
                         VALUES (?,?,?,?,?,?,?,?,?' . str_repeat(',?', count($xc)) . ')');
    foreach ($in['rows'] ?? [] as $r) {
        $x = array_map(function ($c) use ($r) { return isset($r[$c]) && is_numeric($r[$c]) ? $r[$c] + 0 : null; }, $xc);
        $st->execute(array_merge([(int)$r['run'], (int)$r['t'], $r['soc'], $r['soh'], $r['T'],
                                  $r['I'], (int)$r['st'], (int)$r['tec'], (int)$r['rec']], $x));
        $n++;
    }

    $se = $pdo->prepare('INSERT INTO events (run_id,t_ms,code,detail) VALUES (?,?,?,?)');
    foreach ($in['events'] ?? [] as $e) {
        $se->execute([(int)$e['run'], (int)$e['t'], substr($e['code'], 0, 32), substr($e['detail'] ?? '', 0, 255)]);
    }

    $re = $pdo->prepare('UPDATE runs SET ended_at=CURRENT_TIMESTAMP, result=?, summary=? WHERE run_id=?');
    foreach ($in['runs'] ?? [] as $r) {
        if ($r['op'] === 'end') $re->execute([$r['result'] === 'PASS' ? 'PASS' : 'FAIL', substr($r['summary'] ?? '', 0, 255), (int)$r['id']]);
    }

    $sl = $pdo->prepare('INSERT INTO link (seq,srv_ms,sent_ms,rx_first_ms,rx_last_ms,n_rows,n_events,
                         uart_missing,uart_bad,jit_ms,prev_rtt_ms,prev_age_ms,resent) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)');
    $sl->execute([$seq, (int)round(microtime(true) * 1000), $iv('sent_ms'), $iv('rx_first'), $iv('rx_last'),
                  $n, count($in['events'] ?? []) + count($in['runs'] ?? []),
                  $iv('uart_missing'), $iv('uart_bad'), $iv('jit_ms'), $iv('prev_rtt_ms'), $iv('prev_age_ms'),
                  $iv('resent') ?? 0]);

    $pdo->commit();
    out(['ok' => true, 'rows' => $n]);
} catch (Throwable $e) {
    $pdo->rollBack();
    out(['error' => 'db'], 500);
}
