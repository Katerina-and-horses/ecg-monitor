/*
 * ============================================================
 *  ECG MONITOR — ESP32 + AD8232
 *  WiFi AP + WebSocket + встроенный веб-интерфейс
 *  Поддержка людей и лошадей
 *  v2.0 — управление записями, переименование, сравнение
 * ============================================================
 *
 *  ПОДКЛЮЧЕНИЕ AD8232 → ESP32:
 *    OUTPUT  → GPIO34  (только ADC, не через Flash-пины!)
 *    LO+     → GPIO26
 *    LO-     → GPIO25
 *    SDN     → не подключён (AD8232 всегда активен)
 *    3V3     → 3.3V
 *    GND     → GND
 *
 *  ЭЛЕКТРОДЫ (цвета IEC):
 *    🔴 Красный  → RA (правая рука / холка)
 *    🟡 Жёлтый   → LA (левая рука / левый бок)
 *    🟢 Зелёный  → RL (правая нога / брюхо)
 *
 *  ЗАВИСИМОСТИ (установить через Library Manager):
 *    - arduinoWebSockets by Markus Sattler (v2.x)
 *
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ============================================================
//  КОНФИГУРАЦИЯ
// ============================================================

#define ECG_PIN      34
#define LO_PLUS      26
#define LO_MINUS     25

#define SAMPLE_RATE  250
#define BATCH_SIZE   10

const char* AP_SSID = "ECG_Monitor";
const char* AP_PASS = "ecg12345";

// ============================================================
//  ОБЪЕКТЫ
// ============================================================

WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ============================================================
//  БУФЕР / ТАЙМЕР
// ============================================================

volatile uint16_t adcBuf[BATCH_SIZE];
volatile uint8_t  adcIdx   = 0;
volatile bool     pktReady = false;

portMUX_TYPE timerMux  = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t*  sampTimer = NULL;

volatile bool sampleTick = false;
void IRAM_ATTR onTimer() { sampleTick = true; }

// ============================================================
//  HTML / JS (встроен в прошивку)
// ============================================================

const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>ECG Monitor</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700&display=swap');

  :root {
    --green:  #00ff88;
    --cyan:   #00e5ff;
    --red:    #ff3b3b;
    --amber:  #ffb300;
    --bg:     #060a06;
    --bg2:    #0c110c;
    --border: #1a2a1a;
    --dim:    #2a3a2a;
    --text:   #c8d8c8;
    --font:   'Share Tech Mono', monospace;
    --title:  'Orbitron', monospace;
    --t0:     #00ff88;
    --t1:     #00e5ff;
    --t2:     #ffb300;
  }

  * { margin:0; padding:0; box-sizing:border-box; -webkit-tap-highlight-color:transparent; }

  html, body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--font);
    height: 100%;
    overflow: hidden;
    overscroll-behavior: none;
    display: flex;
    flex-direction: column;
  }

  /* ── ШАПКА ── */
  #header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 14px;
    background: var(--bg2);
    border-bottom: 1px solid var(--border);
    height: 52px;
    flex-shrink: 0;
  }
  #appTitle { font-family: var(--title); font-size: 14px; font-weight: 700; color: var(--green); letter-spacing: 3px; text-shadow: 0 0 12px var(--green); }
  #appSub   { font-size: 9px; color: var(--dim); letter-spacing: 1px; margin-top: 2px; }

  #leadBlock { display: flex; align-items: center; gap: 8px; }
  #leadDot {
    width: 10px; height: 10px; border-radius: 50%;
    background: var(--red); box-shadow: 0 0 10px var(--red);
    transition: background 0.3s, box-shadow 0.3s; flex-shrink: 0;
  }
  #leadDot.ok   { background: var(--green); box-shadow: 0 0 10px var(--green); animation: breathe 2s ease-in-out infinite; }
  #leadDot.conn { background: var(--amber); box-shadow: 0 0 10px var(--amber); }
  @keyframes breathe { 0%,100%{opacity:1} 50%{opacity:0.5} }
  #leadText { font-size: 10px; color: var(--dim); letter-spacing: 1px; max-width: 80px; line-height: 1.3; }

  #bpmBlock { text-align: right; }
  #bpmVal   { font-family: var(--title); font-size: 28px; color: var(--cyan); text-shadow: 0 0 16px var(--cyan); line-height: 1; letter-spacing: 2px; }
  #bpmUnit  { font-size: 9px; color: var(--dim); letter-spacing: 2px; }
  #bpmVal.pulse { animation: bpmFlash 0.15s ease-out; }
  @keyframes bpmFlash { 0%{color:#fff;text-shadow:0 0 30px #fff} 100%{color:var(--cyan);text-shadow:0 0 16px var(--cyan)} }

  /* ── ВКЛАДКИ ── */
  #tabBar { display: flex; background: var(--bg2); border-bottom: 1px solid var(--border); flex-shrink: 0; }
  .tab {
    flex: 1; padding: 9px 4px;
    background: transparent; border: none; border-bottom: 2px solid transparent;
    color: var(--dim); font-family: var(--font); font-size: 11px; letter-spacing: 2px;
    cursor: pointer; transition: color 0.2s, border-color 0.2s; -webkit-appearance: none;
  }
  .tab.active { color: var(--green); border-bottom-color: var(--green); }
  #fileCount { font-size: 9px; color: var(--amber); margin-left: 3px; }

  /* ── МОНИТОР ── */
  #monitorSection { flex: 1; display: flex; flex-direction: column; min-height: 0; }
  #ecgWrap { position: relative; background: #000; flex: 1; overflow: hidden; min-height: 0; }
  #ecgCanvas { display: block; width: 100%; height: 100%; image-rendering: pixelated; }
  #overlay { position: absolute; top: 8px; left: 10px; font-size: 10px; color: #2a4a2a; letter-spacing: 1px; pointer-events: none; line-height: 1.6; }

  #controls {
    display: flex; align-items: center; gap: 8px; padding: 8px 12px;
    background: var(--bg2); border-top: 1px solid var(--border);
    flex-shrink: 0; flex-wrap: wrap; min-height: 52px;
  }

  .btn {
    padding: 7px 12px; border: 1px solid currentColor; border-radius: 4px;
    background: transparent; font-family: var(--font); font-size: 11px;
    letter-spacing: 1px; cursor: pointer; transition: background 0.15s, box-shadow 0.15s;
    white-space: nowrap; -webkit-appearance: none;
  }
  .btn:active { filter: brightness(1.4); }

  #recBtn  { color: var(--red); }
  #recBtn.rec { background: rgba(255,59,59,0.15); box-shadow: 0 0 8px var(--red); animation: recPulse 1.2s infinite; }
  @keyframes recPulse { 0%,100%{box-shadow:0 0 8px var(--red)} 50%{box-shadow:0 0 20px var(--red)} }
  #saveBtn  { color: var(--green); }
  #saveBtn:disabled { opacity: 0.25; pointer-events: none; }
  #modeBtn  { color: var(--cyan); }
  #modeBtn.horse { background: rgba(0,229,255,0.1); }

  #gainGroup { display: flex; align-items: center; gap: 6px; margin-left: auto; }
  #gainGroup label { font-size: 10px; color: var(--dim); letter-spacing: 1px; }
  #gainRange { -webkit-appearance: none; width: 80px; height: 3px; border-radius: 2px; background: var(--border); outline: none; cursor: pointer; }
  #gainRange::-webkit-slider-thumb { -webkit-appearance: none; width: 14px; height: 14px; border-radius: 50%; background: var(--green); box-shadow: 0 0 6px var(--green); }
  #gainLabel { font-size: 11px; color: var(--green); min-width: 24px; }

  /* ── ЗАПИСИ ── */
  #filesSection { flex: 1; display: none; flex-direction: column; min-height: 0; overflow: hidden; }

  #fileListPane { flex-shrink: 0; max-height: 44%; overflow-y: auto; border-bottom: 1px solid var(--border); }
  #fileListHeader {
    display: flex; align-items: center; gap: 8px; padding: 8px 12px;
    background: var(--bg2); border-bottom: 1px solid var(--border);
    position: sticky; top: 0; z-index: 1;
  }
  #fileListHeader .lh-title { font-family: var(--title); font-size: 11px; color: var(--green); letter-spacing: 2px; }
  #fileListHeader .lh-hint  { font-size: 9px; color: var(--dim); flex: 1; }
  #clearCompBtn {
    color: var(--dim); font-size: 10px; padding: 4px 8px; border: 1px solid var(--border);
    border-radius: 3px; background: transparent; font-family: var(--font); cursor: pointer; -webkit-appearance: none;
  }
  #fileList { padding: 2px 0; }

  .fileRow {
    display: flex; align-items: center; gap: 8px; padding: 9px 12px;
    border-bottom: 1px solid var(--border); cursor: pointer;
    transition: background 0.15s; border-left: 3px solid transparent;
  }
  .fileRow:active { background: rgba(0,255,136,0.04); }
  .fileRow.sel0 { border-left-color: var(--t0); }
  .fileRow.sel1 { border-left-color: var(--t1); }
  .fileRow.sel2 { border-left-color: var(--t2); }

  .fChk {
    width: 17px; height: 17px; border: 1px solid var(--dim); border-radius: 3px;
    display: flex; align-items: center; justify-content: center; flex-shrink: 0;
    font-size: 9px; font-family: var(--title); transition: border-color 0.2s, background 0.2s;
  }
  .fChk.c0 { border-color: var(--t0); background: rgba(0,255,136,0.12); color: var(--t0); }
  .fChk.c1 { border-color: var(--t1); background: rgba(0,229,255,0.12); color: var(--t1); }
  .fChk.c2 { border-color: var(--t2); background: rgba(255,179,0,0.12);  color: var(--t2); }

  .fName { font-size: 12px; color: var(--text); flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .fMeta { font-size: 9px; color: var(--dim); line-height: 1.6; text-align: right; flex-shrink: 0; min-width: 60px; }
  .fBpm  { font-family: var(--title); font-size: 11px; color: var(--dim); }
  .fBpm.c0 { color: var(--t0); } .fBpm.c1 { color: var(--t1); } .fBpm.c2 { color: var(--t2); }

  .fActs { display: flex; gap: 2px; flex-shrink: 0; }
  .iBtn  { background: transparent; border: none; color: var(--dim); font-size: 14px; cursor: pointer; padding: 3px 5px; -webkit-appearance: none; line-height: 1; border-radius: 3px; }
  .iBtn:active { background: rgba(255,255,255,0.07); color: var(--text); }
  .iBtn.del:active { color: var(--red); }

  .emptyMsg { padding: 36px 20px; text-align: center; color: var(--dim); font-size: 11px; line-height: 2.2; letter-spacing: 1px; }

  /* ── АНАЛИЗ / СРАВНЕНИЕ ── */
  #analysisPane { flex: 1; display: flex; flex-direction: column; min-height: 0; overflow: hidden; }
  #compWrap { flex: 1; background: #000; position: relative; min-height: 0; overflow: hidden; }
  #compCanvas { display: block; width: 100%; height: 100%; }
  #compPlaceholder { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; color: var(--dim); font-size: 11px; letter-spacing: 2px; flex-direction: column; gap: 8px; }
  #compPlaceholder span { font-size: 24px; }

  #statsPane {
    flex-shrink: 0; display: flex; gap: 0;
    overflow-x: auto; background: var(--bg2); border-top: 1px solid var(--border);
    min-height: 0;
  }
  .statCard { flex: 1; min-width: 88px; padding: 8px 10px; border-right: 1px solid var(--border); }
  .statCard:last-child { border-right: none; }
  .stTitle { font-size: 8px; color: var(--dim); letter-spacing: 1px; margin-bottom: 4px; text-transform: uppercase; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .stVal   { font-family: var(--title); font-size: 16px; line-height: 1.1; }
  .stSub   { font-size: 9px; color: var(--dim); margin-top: 2px; }

  /* ── МОДАЛЬНОЕ ОКНО: СОХРАНЕНИЕ ── */
  #renameModal {
    position: fixed; inset: 0;
    background: rgba(0,0,0,0.88);
    display: none; align-items: center; justify-content: center;
    z-index: 200; padding: 20px;
  }
  #renameBox {
    background: var(--bg2); border: 1px solid var(--border);
    border-radius: 10px; padding: 22px 20px;
    width: 100%; max-width: 320px;
  }
  #renameBox h3 { font-family: var(--title); font-size: 11px; color: var(--green); letter-spacing: 3px; margin-bottom: 16px; text-shadow: 0 0 8px var(--green); }
  .fRow { margin-bottom: 12px; }
  .fRow label { display: block; font-size: 10px; color: var(--dim); letter-spacing: 1px; margin-bottom: 5px; }
  .fRow input[type=text] {
    width: 100%; background: #000; border: 1px solid var(--border); border-radius: 4px;
    padding: 9px 10px; color: var(--text); font-family: var(--font); font-size: 13px;
    outline: none; -webkit-appearance: none;
  }
  .fRow input[type=text]:focus { border-color: var(--green); }
  #recInfo { font-size: 10px; color: var(--dim); line-height: 1.9; margin-bottom: 14px; padding: 8px 10px; background: #000; border-radius: 4px; border: 1px solid var(--border); }
  #recInfo b { color: var(--text); font-weight: normal; }
  .mBtns { display: flex; gap: 8px; }
  .mBtns .btn { flex: 1; text-align: center; padding: 9px 6px; font-size: 10px; }
  .btn-ghost   { color: var(--dim); }
  .btn-dl      { color: var(--cyan); }
  .btn-primary { color: #000 !important; background: var(--green); border-color: var(--green); font-weight: bold; }
  .btn-primary:active { filter: brightness(0.85); }

  /* ── СТАРТОВЫЙ ЭКРАН ── */
  #splash {
    position: fixed; inset: 0; background: rgba(0,0,0,0.93);
    display: flex; align-items: center; justify-content: center; z-index: 100; padding: 20px;
  }
  #splashBox { background: var(--bg2); border: 1px solid var(--border); border-radius: 12px; padding: 28px 24px; max-width: 340px; width: 100%; }
  #splashBox h2 { font-family: var(--title); color: var(--green); font-size: 18px; letter-spacing: 4px; margin-bottom: 16px; text-align: center; text-shadow: 0 0 12px var(--green); }
  .splashRow { display: flex; align-items: center; gap: 10px; margin: 8px 0; font-size: 13px; color: var(--text); }
  .splashRow .dot { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  .dot-r { background: var(--red); } .dot-y { background: #ffdd00; } .dot-g { background: var(--green); }
  .splashDiv { border: none; border-top: 1px solid var(--border); margin: 14px 0; }
  .splashInfo { font-size: 12px; color: var(--dim); line-height: 1.8; margin-bottom: 6px; }
  .splashInfo b { color: var(--cyan); }
  #splashBtn {
    display: block; width: 100%; margin-top: 18px; padding: 11px;
    background: var(--green); color: #000; border: none; border-radius: 6px;
    font-family: var(--title); font-size: 13px; letter-spacing: 2px; cursor: pointer; font-weight: 700; -webkit-appearance: none;
  }
  #splashBtn:active { filter: brightness(0.85); }
</style>
</head>
<body>

<!-- ШАПКА -->
<div id="header">
  <div>
    <div id="appTitle">ECG MONITOR</div>
    <div id="appSub">ESP32 · AD8232 · 250 HZ</div>
  </div>
  <div id="leadBlock">
    <div id="leadDot" class="conn"></div>
    <div id="leadText">подключение...</div>
  </div>
  <div id="bpmBlock">
    <div id="bpmVal">---</div>
    <div id="bpmUnit">УД/МИН</div>
  </div>
</div>

<!-- ВКЛАДКИ -->
<div id="tabBar">
  <button class="tab active" id="tabMonitor" onclick="switchTab('monitor')">⬤ МОНИТОР</button>
  <button class="tab"        id="tabFiles"   onclick="switchTab('files')">📁 ЗАПИСИ<span id="fileCount"></span></button>
</div>

<!-- МОНИТОР -->
<div id="monitorSection">
  <div id="ecgWrap">
    <canvas id="ecgCanvas"></canvas>
    <div id="overlay">25 MM/S · 10 MM/MV</div>
  </div>
  <div id="controls">
    <button class="btn" id="recBtn"  onclick="toggleRec()">⬤ ЗАПИСЬ</button>
    <button class="btn" id="saveBtn" onclick="showRenameModal()" disabled>💾 СОХРАНИТЬ</button>
    <button class="btn" id="modeBtn" onclick="toggleMode()">🐴 ЛОШАДЬ</button>
    <div id="gainGroup">
      <label>УСИЛ</label>
      <input type="range" id="gainRange" min="1" max="20" value="5"
             oninput="gainLabel.textContent=this.value+'×'; gain=+this.value;">
      <span id="gainLabel">5×</span>
    </div>
  </div>
</div>

<!-- ЗАПИСИ -->
<div id="filesSection">
  <div id="fileListPane">
    <div id="fileListHeader">
      <span class="lh-title">ЗАПИСИ</span>
      <span class="lh-hint">выберите до 3 для сравнения</span>
      <button id="clearCompBtn" onclick="clearCompare()">СБРОСИТЬ</button>
    </div>
    <div id="fileList">
      <div class="emptyMsg">записей нет<br>нажмите ⬤ ЗАПИСЬ на вкладке МОНИТОР</div>
    </div>
  </div>
  <div id="analysisPane">
    <div id="compWrap">
      <canvas id="compCanvas"></canvas>
      <div id="compPlaceholder"><span>📈</span>ВЫБЕРИТЕ ЗАПИСЬ ВЫШЕ</div>
    </div>
    <div id="statsPane"></div>
  </div>
</div>

<!-- МОДАЛЬНОЕ ОКНО: СОХРАНЕНИЕ / ПЕРЕИМЕНОВАНИЕ -->
<div id="renameModal">
  <div id="renameBox">
    <h3>СОХРАНИТЬ ЗАПИСЬ</h3>
    <div class="fRow">
      <label>НАЗВАНИЕ ФАЙЛА</label>
      <input type="text" id="recNameInput" placeholder="Звёздочка_01">
    </div>
    <div id="recInfo">—</div>
    <div class="mBtns">
      <button class="btn btn-ghost"   onclick="discardRec()">✕ УДАЛИТЬ</button>
      <button class="btn btn-dl"      onclick="downloadRec()">↓ CSV</button>
      <button class="btn btn-primary" onclick="saveRec()">💾 СОХРАНИТЬ</button>
    </div>
  </div>
</div>

<!-- СТАРТОВЫЙ ЭКРАН -->
<div id="splash">
  <div id="splashBox">
    <h2>ECG MONITOR</h2>
    <div class="splashRow"><div class="dot dot-r"></div><span>Красный → RA (правая рука / холка)</span></div>
    <div class="splashRow"><div class="dot dot-y"></div><span>Жёлтый → LA (левая рука / левый бок)</span></div>
    <div class="splashRow"><div class="dot dot-g"></div><span>Зелёный → RL (правая нога / брюхо)</span></div>
    <hr class="splashDiv">
    <div class="splashInfo">WiFi: <b>ECG_Monitor</b> · Пароль: <b>ecg12345</b><br>Адрес: <b>192.168.4.1</b></div>
    <button id="splashBtn" onclick="startApp()">НАЧАТЬ</button>
  </div>
</div>

<script>
// ====================================================
//  КОНСТАНТЫ
// ====================================================
const WS_PORT      = 81;
const FS           = 250;
const PX_PER_S     = 200;
const ERASE_W      = 30;
const TRACK_COLORS = ['#00ff88', '#00e5ff', '#ffb300'];

// ====================================================
//  СОСТОЯНИЕ МОНИТОРА
// ====================================================
let canvas, ctx, W, H;
let gain      = 5;
let horseMode = false;
let recording = false;
let recData   = [];      // сырые uint16
let recStart  = 0;
let ws, wsReady = false;
let pxPerSample, cursorX = 0, prevY = null;

const HR_RANGE = { human: [30, 220], horse: [20, 100] };
const REFRACT  = { human: 350, horse: 700 };
let peaks = [], lastPeakMs = 0, bpm = 0;
let baseline = 2048;
const ALPHA_BL = 0.002;
let peakSmooth = 0, peakMax = 0;

// ====================================================
//  СОСТОЯНИЕ ФАЙЛОВ
// ====================================================
let selectedIds = [];   // до 3 id записей для сравнения
let allRecs     = [];   // кэш всех записей
let pendingMode = false;

// ====================================================
//  INDEXEDDB
// ====================================================
const DB_NAME = 'ECG_DB', DB_VER = 1, STORE = 'recordings';
let db;

function openDB() {
  return new Promise((res, rej) => {
    const req = indexedDB.open(DB_NAME, DB_VER);
    req.onupgradeneeded = e => {
      const d = e.target.result;
      if (!d.objectStoreNames.contains(STORE))
        d.createObjectStore(STORE, { keyPath: 'id', autoIncrement: true });
    };
    req.onsuccess = e => { db = e.target.result; res(db); };
    req.onerror   = rej;
  });
}

function dbStore(mode) { return db.transaction(STORE, mode).objectStore(STORE); }

function dbGetAll() {
  return new Promise((res, rej) => {
    const r = dbStore('readonly').getAll();
    r.onsuccess = () => res(r.result); r.onerror = rej;
  });
}
function dbGet(id) {
  return new Promise((res, rej) => {
    const r = dbStore('readonly').get(id);
    r.onsuccess = () => res(r.result); r.onerror = rej;
  });
}
function dbAdd(rec) {
  return new Promise((res, rej) => {
    const r = dbStore('readwrite').add(rec);
    r.onsuccess = () => res(r.result); r.onerror = rej;
  });
}
function dbPut(rec) {
  return new Promise((res, rej) => {
    const r = dbStore('readwrite').put(rec);
    r.onsuccess = () => res(r.result); r.onerror = rej;
  });
}
function dbDelete(id) {
  return new Promise((res, rej) => {
    const r = dbStore('readwrite').delete(id);
    r.onsuccess = res; r.onerror = rej;
  });
}

// ====================================================
//  ЗАПУСК
// ====================================================
async function startApp() {
  document.getElementById('splash').style.display = 'none';
  await openDB();
  initCanvas();
  connectWS();
  await refreshFileCount();
}

// ====================================================
//  ВКЛАДКИ
// ====================================================
function switchTab(tab) {
  const mon = tab === 'monitor';
  document.getElementById('monitorSection').style.display = mon ? 'flex' : 'none';
  document.getElementById('filesSection').style.display   = mon ? 'none' : 'flex';
  document.getElementById('tabMonitor').classList.toggle('active',  mon);
  document.getElementById('tabFiles').classList.toggle('active', !mon);
  if (!mon) loadFileList();
}

// ====================================================
//  КАНВАС МОНИТОРА
// ====================================================
function initCanvas() {
  canvas = document.getElementById('ecgCanvas');
  ctx    = canvas.getContext('2d');
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);
  clearCanvas();
}

function resizeCanvas() {
  const wrap = document.getElementById('ecgWrap');
  W = wrap.clientWidth; H = wrap.clientHeight;
  canvas.width  = W; canvas.height = H;
  pxPerSample   = PX_PER_S / FS;
  clearCanvas(); cursorX = 0; prevY = null;
}

function clearCanvas() {
  ctx.fillStyle = '#000'; ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, W, H);
}

function drawGrid(c, w, h) {
  const big = PX_PER_S * 0.2, sm = big / 5;
  c.lineWidth = 0.5;
  for (let x = 0; x < w; x += sm) {
    c.strokeStyle = (Math.abs(x - Math.round(x/big)*big) < 0.5) ? '#1c2e1c' : '#0e180e';
    c.beginPath(); c.moveTo(x, 0); c.lineTo(x, h); c.stroke();
  }
  for (let y = 0; y < h; y += sm) {
    c.strokeStyle = (Math.abs(y - Math.round(y/big)*big) < 0.5) ? '#1c2e1c' : '#0e180e';
    c.beginPath(); c.moveTo(0, y); c.lineTo(w, y); c.stroke();
  }
  c.strokeStyle = '#0f200f'; c.lineWidth = 1;
  c.beginPath(); c.moveTo(0, h/2); c.lineTo(w, h/2); c.stroke();
}

function redrawGridStrip(x0, sw) {
  const big = PX_PER_S * 0.2, sm = big / 5;
  ctx.lineWidth = 0.5;
  for (let x = Math.floor(x0/sm)*sm; x <= x0+sw; x += sm) {
    if (x < x0) continue;
    ctx.strokeStyle = (Math.abs(x - Math.round(x/big)*big) < 1) ? '#1c2e1c' : '#0e180e';
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
  }
  for (let y = 0; y < H; y += sm) {
    ctx.strokeStyle = (Math.abs(y - Math.round(y/big)*big) < 1) ? '#1c2e1c' : '#0e180e';
    ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x0+sw, y); ctx.stroke();
  }
  ctx.strokeStyle = '#0f200f'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(x0, H/2); ctx.lineTo(x0+sw, H/2); ctx.stroke();
}

// ====================================================
//  WEBSOCKET
// ====================================================
function connectWS() {
  setLead('conn', 'подключение...');
  ws = new WebSocket(`ws://${location.hostname}:${WS_PORT}`);
  ws.binaryType = 'arraybuffer';
  ws.onopen  = () => { wsReady = true; setLead('conn', 'ожидание данных'); };
  ws.onclose = () => { wsReady = false; setLead('off', 'нет связи'); setTimeout(connectWS, 2000); };
  ws.onerror = () => ws.close();
  ws.onmessage = evt => {
    const dv  = new DataView(evt.data);
    const n   = (evt.data.byteLength - 1) / 2;
    const lo  = !!(dv.getUint8(evt.data.byteLength - 1) & 0x01);
    setLead(lo ? 'off' : 'ok', lo ? 'нет контакта' : 'сигнал OK');
    for (let i = 0; i < n; i++) processSample(dv.getUint16(i * 2, true), lo);
  };
}

// ====================================================
//  ОБРАБОТКА ВЫБОРКИ
// ====================================================
function processSample(raw, leadOff) {
  baseline += ALPHA_BL * (raw - baseline);
  const v = (raw - baseline) / 2048.0;
  if (recording && !leadOff) recData.push(raw);
  if (!leadOff) detectPeak(v);
  drawSample(v, leadOff);
}

// ====================================================
//  ОТРИСОВКА (бегущий курсор)
// ====================================================
function drawSample(v, leadOff) {
  const y = H/2 - v * (H * 0.4) * (gain / 5);
  const x = cursorX;
  ctx.fillStyle = '#000';
  ctx.fillRect(x + 1, 0, ERASE_W, H);
  redrawGridStrip(x + 1, ERASE_W);

  if (prevY !== null && !leadOff) {
    ctx.beginPath();
    ctx.strokeStyle = '#00ff66'; ctx.lineWidth = 1.8;
    ctx.shadowColor = '#00ff66'; ctx.shadowBlur = 5;
    ctx.moveTo(x - pxPerSample, prevY); ctx.lineTo(x, y); ctx.stroke();
    ctx.shadowBlur = 0;
  } else if (leadOff && prevY !== null) {
    ctx.beginPath(); ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1;
    ctx.setLineDash([3, 5]);
    ctx.moveTo(x - pxPerSample, prevY); ctx.lineTo(x, H/2); ctx.stroke();
    ctx.setLineDash([]);
  }
  prevY    = leadOff ? null : y;
  cursorX += pxPerSample;
  if (cursorX >= W) { cursorX = 0; prevY = null; }
}

// ====================================================
//  ДЕТЕКТОР BPM
// ====================================================
function detectPeak(v) {
  peakSmooth = peakSmooth * 0.97 + v * 0.03;
  peakMax    = Math.max(peakMax * 0.9995, Math.abs(v));
  const thr    = Math.max(0.15, peakMax * 0.6);
  const refMs  = horseMode ? REFRACT.horse : REFRACT.human;
  const now    = Date.now();
  if (v > thr && (now - lastPeakMs) > refMs) {
    lastPeakMs = now;
    peaks.push(now);
    peaks = peaks.filter(t => now - t < 10000);
    if (peaks.length >= 3) {
      const ivs  = peaks.slice(1).map((t,i) => t - peaks[i]);
      const avg  = ivs.reduce((a,b)=>a+b,0) / ivs.length;
      const nb   = Math.round(60000 / avg);
      const rng  = horseMode ? HR_RANGE.horse : HR_RANGE.human;
      if (nb >= rng[0] && nb <= rng[1]) {
        bpm = nb;
        const el = document.getElementById('bpmVal');
        el.textContent = bpm;
        el.classList.remove('pulse'); void el.offsetWidth; el.classList.add('pulse');
      }
    }
  }
}

// ====================================================
//  СТАТУС ОТВЕДЕНИЙ
// ====================================================
function setLead(state, text) {
  const dot = document.getElementById('leadDot');
  dot.className = '';
  if (state === 'ok')   dot.classList.add('ok');
  if (state === 'conn') dot.classList.add('conn');
  document.getElementById('leadText').textContent = text;
}

// ====================================================
//  ЗАПИСЬ
// ====================================================
function toggleRec() {
  recording = !recording;
  const btn  = document.getElementById('recBtn');
  const save = document.getElementById('saveBtn');
  if (recording) {
    recData = []; recStart = Date.now();
    btn.textContent = '■ СТОП'; btn.classList.add('rec'); save.disabled = true;
  } else {
    btn.textContent = '⬤ ЗАПИСЬ'; btn.classList.remove('rec');
    save.disabled = recData.length === 0;
  }
}

// ====================================================
//  МОДАЛЬНОЕ ОКНО СОХРАНЕНИЯ
// ====================================================
function showRenameModal() {
  if (!recData.length) return;
  pendingMode = horseMode;
  const dur = recData.length / FS;
  const dt  = new Date();
  const defName = (horseMode ? 'Лошадь' : 'Пациент') +
    '_' + dt.toLocaleDateString('ru-RU').replace(/\./g,'');
  document.getElementById('recNameInput').value = defName;
  document.getElementById('recInfo').innerHTML =
    `Длительность: <b>${fmtDur(dur)}</b><br>` +
    `Выборок: <b>${recData.length}</b><br>` +
    `Режим: <b>${horseMode ? '🐴 Лошадь' : '👤 Человек'}</b><br>` +
    `Время: <b>${dt.toLocaleString('ru-RU')}</b>`;
  document.getElementById('renameModal').style.display = 'flex';
  const inp = document.getElementById('recNameInput');
  inp.focus(); inp.select();
}

function hideRenameModal() {
  document.getElementById('renameModal').style.display = 'none';
}

async function saveRec() {
  const name = document.getElementById('recNameInput').value.trim() || 'ECG_' + Date.now();
  const rec  = {
    name, date: Date.now(),
    mode:     pendingMode ? 'horse' : 'human',
    fs:       FS,
    samples:  Array.from(recData),
    duration: recData.length / FS
  };
  rec.bpm = analyzeSignal(rec.samples, rec.mode).bpm;
  try {
    await dbAdd(rec);
    hideRenameModal();
    recData = [];
    document.getElementById('saveBtn').disabled = true;
    await refreshFileCount();
  } catch(e) { alert('Ошибка: ' + e); }
}

function downloadRec() {
  const name = document.getElementById('recNameInput').value.trim() || 'ECG_' + Date.now();
  exportCSV(recData, name, pendingMode ? 'horse' : 'human');
  hideRenameModal();
}

function discardRec() {
  if (!confirm('Удалить запись без сохранения?')) return;
  recData = [];
  document.getElementById('saveBtn').disabled = true;
  hideRenameModal();
}

// ====================================================
//  ЭКСПОРТ CSV
// ====================================================
function exportCSV(samples, name, mode) {
  const hdr  = `time_s,raw_adc\n# name=${name} mode=${mode} fs=${FS} device=ESP32+AD8232\n`;
  const rows = samples.map((v, i) => `${(i/FS).toFixed(4)},${v}`).join('\n');
  const blob = new Blob([hdr + rows], { type: 'text/csv' });
  const a    = Object.assign(document.createElement('a'), {
    href: URL.createObjectURL(blob),
    download: name.replace(/[^\wа-яА-Я\-]/g,'_') + '.csv'
  });
  a.click();
}

// ====================================================
//  СПИСОК ФАЙЛОВ
// ====================================================
async function refreshFileCount() {
  try {
    const r = await dbGetAll();
    const el = document.getElementById('fileCount');
    el.textContent = r.length ? ` (${r.length})` : '';
  } catch(e) {}
}

async function loadFileList() {
  try {
    allRecs = (await dbGetAll()).sort((a, b) => b.date - a.date);
    renderFileList(allRecs);
    updateCompView();
  } catch(e) { console.error(e); }
}

function renderFileList(recs) {
  const el = document.getElementById('fileList');
  if (!recs.length) {
    el.innerHTML = '<div class="emptyMsg">записей нет<br>нажмите ⬤ ЗАПИСЬ · 💾 СОХРАНИТЬ</div>';
    return;
  }
  el.innerHTML = recs.map(rec => {
    const si  = selectedIds.indexOf(rec.id);
    const sel = si !== -1;
    return `
      <div class="fileRow ${sel ? 'sel'+si : ''}" onclick="toggleSel(${rec.id})">
        <div class="fChk ${sel ? 'c'+si : ''}">${sel ? si+1 : ''}</div>
        <div style="flex:1;overflow:hidden">
          <div class="fName">${esc(rec.name)}</div>
          <div class="fMeta">${fmtDate(rec.date)} · ${fmtDur(rec.duration)} · ${rec.mode==='horse'?'🐴':'👤'}</div>
        </div>
        <div class="fBpm ${sel?'c'+si:''}">${rec.bpm ? rec.bpm+' уд' : '—'}</div>
        <div class="fActs" onclick="event.stopPropagation()">
          <button class="iBtn" title="Переименовать" onclick="renameRec(${rec.id})">✎</button>
          <button class="iBtn" title="Скачать CSV"   onclick="dlRec(${rec.id})">↓</button>
          <button class="iBtn del" title="Удалить"   onclick="delRec(${rec.id})">✕</button>
        </div>
      </div>`;
  }).join('');
}

function toggleSel(id) {
  const idx = selectedIds.indexOf(id);
  if (idx !== -1) selectedIds.splice(idx, 1);
  else { if (selectedIds.length >= 3) selectedIds.shift(); selectedIds.push(id); }
  renderFileList(allRecs);
  updateCompView();
}

function clearCompare() {
  selectedIds = [];
  renderFileList(allRecs);
  updateCompView();
}

async function dlRec(id) {
  const r = await dbGet(id);
  if (r) exportCSV(r.samples, r.name, r.mode);
}

async function delRec(id) {
  if (!confirm('Удалить эту запись?')) return;
  await dbDelete(id);
  selectedIds = selectedIds.filter(i => i !== id);
  allRecs     = allRecs.filter(r => r.id !== id);
  renderFileList(allRecs);
  await refreshFileCount();
  updateCompView();
}

async function renameRec(id) {
  const rec = allRecs.find(r => r.id === id);
  if (!rec) return;
  const nw = prompt('Новое название:', rec.name);
  if (!nw || nw.trim() === rec.name) return;
  rec.name = nw.trim();
  try {
    await dbPut(rec);
    allRecs = allRecs.map(r => r.id === id ? rec : r);
    renderFileList(allRecs);
    updateCompView();
  } catch(e) { alert('Ошибка: ' + e); }
}

// ====================================================
//  АНАЛИЗ / СРАВНЕНИЕ
// ====================================================
async function updateCompView() {
  const ph = document.getElementById('compPlaceholder');
  if (!selectedIds.length) {
    ph.style.display = 'flex';
    document.getElementById('statsPane').innerHTML = '';
    clearCompCanvas();
    return;
  }
  ph.style.display = 'none';
  const recs = selectedIds.map(id => allRecs.find(r => r.id === id)).filter(Boolean);
  drawCompCanvas(recs);
  renderStats(recs);
}

function clearCompCanvas() {
  const wrap = document.getElementById('compWrap');
  const cc   = document.getElementById('compCanvas');
  cc.width = wrap.clientWidth; cc.height = wrap.clientHeight;
  const c = cc.getContext('2d');
  c.fillStyle = '#000'; c.fillRect(0, 0, cc.width, cc.height);
}

function drawCompCanvas(recs) {
  const wrap = document.getElementById('compWrap');
  const cc   = document.getElementById('compCanvas');
  const CW   = wrap.clientWidth, CH = wrap.clientHeight;
  cc.width = CW; cc.height = CH;
  const c = cc.getContext('2d');

  c.fillStyle = '#000'; c.fillRect(0, 0, CW, CH);
  drawGrid(c, CW, CH);

  if (!recs.length) return;

  // Показываем до 10 сек от каждой записи
  const showSamples = Math.min(FS * 10, ...recs.map(r => r.samples.length));
  const step        = Math.max(1, Math.floor(showSamples / CW));
  const n           = recs.length;

  recs.forEach((rec, ti) => {
    const col     = TRACK_COLORS[ti];
    const slice   = rec.samples.slice(0, showSamples);

    // Убираем дрейф изолинии
    let bl = slice.reduce((a,b)=>a+b,0)/slice.length;
    const vals = slice.map(s => s - bl);
    const maxV = Math.max(...vals.map(Math.abs)) || 1;

    // Вертикальная полоса для этого трека
    const trackH   = CH / n;
    const trackMid = (ti + 0.5) * trackH;
    const amp       = trackH * 0.38;

    // Разделитель
    if (ti > 0) {
      c.strokeStyle = '#1a2a1a'; c.lineWidth = 1; c.setLineDash([3,6]);
      c.beginPath(); c.moveTo(0, ti*trackH); c.lineTo(CW, ti*trackH); c.stroke();
      c.setLineDash([]);
    }

    // Кривая ЭКГ
    c.beginPath();
    c.strokeStyle  = col;
    c.lineWidth    = n === 1 ? 1.8 : 1.3;
    c.shadowColor  = col;
    c.shadowBlur   = n === 1 ? 5 : 2;
    let first = true;
    for (let i = 0; i < slice.length; i += step) {
      const x = (i / showSamples) * CW;
      const y = trackMid - (vals[i] / maxV) * amp;
      first ? c.moveTo(x, y) : c.lineTo(x, y);
      first = false;
    }
    c.stroke();
    c.shadowBlur = 0;

    // Подпись
    c.fillStyle = col; c.font = '9px monospace';
    c.fillText(rec.name.slice(0, 22), 6, trackMid - amp - 4);

    // Временная шкала
    const durShown = Math.min(10, rec.duration);
    c.fillStyle = '#2a3a2a'; c.font = '8px monospace';
    c.fillText('0s', 4, trackMid + amp + 11);
    c.fillText(durShown.toFixed(0)+'s', CW-22, trackMid + amp + 11);
  });
}

// ====================================================
//  АНАЛИЗ СИГНАЛА
// ====================================================
function analyzeSignal(samples, mode) {
  if (!samples || samples.length < 20) return { bpm:0, hrv:0, peaks:0 };

  // Убираем дрейф
  let bl = 2048;
  const vs = samples.map(s => { bl += 0.002*(s-bl); return (s-bl)/2048; });

  // Детекция пиков
  let pm = 0, ps = 0, lp = -999;
  const refSamp = (mode === 'horse' ? 700 : 350) / 1000 * FS;
  const pkIdx   = [];
  vs.forEach((v, i) => {
    ps = ps*0.97 + v*0.03;
    pm = Math.max(pm*0.9995, Math.abs(v));
    if (v > Math.max(0.15, pm*0.6) && (i-lp) > refSamp) { lp = i; pkIdx.push(i); }
  });

  let bpmVal = 0, hrv = 0;
  if (pkIdx.length >= 2) {
    const ivs  = pkIdx.slice(1).map((p,i) => (p - pkIdx[i]) / FS * 1000);
    const avg  = ivs.reduce((a,b)=>a+b,0) / ivs.length;
    bpmVal = Math.round(60000 / avg);
    hrv    = Math.round(Math.sqrt(ivs.reduce((a,b)=>a+(b-avg)**2,0)/ivs.length));
    const rng = mode === 'horse' ? HR_RANGE.horse : HR_RANGE.human;
    if (bpmVal < rng[0] || bpmVal > rng[1]) bpmVal = 0;
  }
  return { bpm: bpmVal, hrv, peaks: pkIdx.length };
}

function renderStats(recs) {
  const pane = document.getElementById('statsPane');
  pane.innerHTML = recs.map((rec, ti) => {
    const col   = TRACK_COLORS[ti];
    const stats = analyzeSignal(rec.samples, rec.mode);
    return `
      <div class="statCard">
        <div class="stTitle" style="color:${col}">${esc(rec.name.slice(0,14))}</div>
        <div class="stVal"   style="color:${col}">${stats.bpm || '—'}</div>
        <div class="stSub">уд/мин · ${rec.mode==='horse'?'🐴':'👤'}</div>
        <div class="stSub">HRV ${stats.hrv ? stats.hrv+'ms' : '—'} · ${stats.peaks} пиков</div>
        <div class="stSub">${fmtDur(rec.duration)}</div>
      </div>`;
  }).join('');
}

// ====================================================
//  РЕЖИМ ЛОШАДЬ / ЧЕЛОВЕК
// ====================================================
function toggleMode() {
  horseMode = !horseMode;
  document.getElementById('modeBtn').textContent = horseMode ? '👤 ЧЕЛОВЕК' : '🐴 ЛОШАДЬ';
  document.getElementById('modeBtn').classList.toggle('horse', horseMode);
  document.getElementById('bpmUnit').textContent = horseMode ? 'УД/МИН 🐴' : 'УД/МИН';
  document.getElementById('bpmVal').textContent  = '---';
  peaks=[]; bpm=0; peakMax=0;
}

// ====================================================
//  УТИЛИТЫ
// ====================================================
function fmtDur(sec) {
  const m = Math.floor(sec/60), s = Math.floor(sec%60);
  return m > 0 ? `${m}м ${s}с` : `${s}с`;
}

function fmtDate(ts) {
  return new Date(ts).toLocaleString('ru-RU', {
    day:'2-digit', month:'2-digit', hour:'2-digit', minute:'2-digit'
  });
}

function esc(s) {
  return s.replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
}
</script>
</body>
</html>
)=====";

// ============================================================
//  WEBSOCKET EVENTS
// ============================================================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Клиент #%d подключился\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Клиент #%d отключился\n", num);
      break;
    default:
      break;
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ECG Monitor v2 Boot ===");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(LO_PLUS,  INPUT);
  pinMode(LO_MINUS, INPUT);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] AP: %s  IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  httpServer.on("/", HTTP_GET, []() {
    httpServer.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  httpServer.begin();
  Serial.println("[HTTP] Запущен на порту 80");

  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println("[WS]   Запущен на порту 81");

  sampTimer = timerBegin(1000000);
  timerAttachInterrupt(sampTimer, &onTimer);
  timerAlarm(sampTimer, 1000000UL / SAMPLE_RATE, true, 0);
  Serial.printf("[ADC]  Таймер %d Гц запущен\n", SAMPLE_RATE);

  Serial.println("=== Готово! http://192.168.4.1 ===\n");
}

// ============================================================
//  LOOP
// ============================================================
static uint8_t packet[BATCH_SIZE * 2 + 1];

void loop() {
  httpServer.handleClient();
  wsServer.loop();

  if (sampleTick) {
    sampleTick = false;
    uint16_t raw = (uint16_t)analogRead(ECG_PIN);
    portENTER_CRITICAL(&timerMux);
    adcBuf[adcIdx++] = raw;
    if (adcIdx >= BATCH_SIZE) { adcIdx = 0; pktReady = true; }
    portEXIT_CRITICAL(&timerMux);
  }

  if (pktReady) {
    pktReady = false;
    bool leadOff = digitalRead(LO_PLUS) | digitalRead(LO_MINUS);

    uint16_t snap[BATCH_SIZE];
    portENTER_CRITICAL(&timerMux);
    memcpy(snap, (const void*)adcBuf, sizeof(snap));
    portEXIT_CRITICAL(&timerMux);

    for (int i = 0; i < BATCH_SIZE; i++) {
      packet[i*2]   = snap[i] & 0xFF;
      packet[i*2+1] = (snap[i] >> 8) & 0xFF;
    }
    packet[BATCH_SIZE * 2] = leadOff ? 0x01 : 0x00;
    wsServer.broadcastBIN(packet, sizeof(packet));
  }
}
