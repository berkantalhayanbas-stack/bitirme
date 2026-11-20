<?php
// POST JSON: {"rows":[...], "events":[...], "runs":[...]}  Header: X-API-Key
require __DIR__ . '/config.php';

if ($_SERVER['REQUEST_METHOD'] !== 'POST') out(['error' => 'POST only'], 405);
if (!hash_equals(API_KEY, $_SERVER['HTTP_X_API_KEY'] ?? '')) out(['error' => 'unauthorized'], 401);

$in = json_decode(file_get_contents('php://input'), true);
if (!is_array($in)) out(['error' => 'bad json'], 400);

$pdo = db();
$pdo->beginTransaction();
try {
    $n = 0;
    // önce başlayan senaryolar: aynı numara yeniden koşuyorsa eski izi sil (canlı mod run 0 korunur)
    $rs = $pdo->prepare('INSERT INTO runs (run_id,name) VALUES (?,?)
                         ON DUPLICATE KEY UPDATE name=VALUES(name), started_at=NOW(), ended_at=NULL, result="RUNNING", summary=NULL');
    $dt = $pdo->prepare('DELETE FROM telemetry WHERE run_id=? AND run_id>0');
    $de = $pdo->prepare('DELETE FROM events WHERE run_id=? AND run_id>0');
    foreach ($in['runs'] ?? [] as $r) {
        if ($r['op'] !== 'start') continue;
        $dt->execute([(int)$r['id']]);  $de->execute([(int)$r['id']]);
        $rs->execute([(int)$r['id'], substr($r['name'], 0, 64)]);
    }

    $st = $pdo->prepare('INSERT INTO telemetry (run_id,t_ms,soc,soh,temp,i_allow,state,tec,rec)
                         VALUES (?,?,?,?,?,?,?,?,?)');
    foreach ($in['rows'] ?? [] as $r) {
        $st->execute([(int)$r['run'], (int)$r['t'], $r['soc'], $r['soh'], $r['T'],
                      $r['I'], (int)$r['st'], (int)$r['tec'], (int)$r['rec']]);
        $n++;
    }

    $se = $pdo->prepare('INSERT INTO events (run_id,t_ms,code,detail) VALUES (?,?,?,?)');
    foreach ($in['events'] ?? [] as $e) {
        $se->execute([(int)$e['run'], (int)$e['t'], substr($e['code'], 0, 32), substr($e['detail'] ?? '', 0, 255)]);
    }

    $re = $pdo->prepare('UPDATE runs SET ended_at=NOW(), result=?, summary=? WHERE run_id=?');
    foreach ($in['runs'] ?? [] as $r) {
        if ($r['op'] === 'end') $re->execute([$r['result'] === 'PASS' ? 'PASS' : 'FAIL', substr($r['summary'] ?? '', 0, 255), (int)$r['id']]);
    }

    $pdo->commit();
    out(['ok' => true, 'rows' => $n]);
} catch (Throwable $e) {
    $pdo->rollBack();
    out(['error' => 'db'], 500);
}
