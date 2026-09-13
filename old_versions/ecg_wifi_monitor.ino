/*
 * ЭКГ Монитор — ESP32 + AD8232
 * Беспроводная передача через WiFi, веб-интерфейс на телефоне
 *
 * БИБЛИОТЕКИ (установить через Arduino Library Manager):
 *   - "ESP Async WebServer" by lacamera
 *   - "AsyncTCP" by dvarrel
 *
 * ПОДКЛЮЧЕНИЕ ПИНОВ:
 *   AD8232 OUTPUT → GPIO 34 (ECG_PIN)
 *   AD8232 LO+    → GPIO 26 (LO_PLUS)
 *   AD8232 LO-    → GPIO 25 (LO_MINUS)
 *
 * КАК ИСПОЛЬЗОВАТЬ:
 *   1. Загрузить прошивку в ESP32
 *   2. Подключить телефон к WiFi "ECG_Monitor" (пароль: ecg12345)
 *   3. Открыть Chrome → http://192.168.4.1
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ===== НАСТРОЙКИ =====
const char* AP_SSID     = "ECG_Monitor";
const char* AP_PASSWORD = "ecg12345";   // минимум 8 символов

const int ECG_PIN  = 34;
const int LO_PLUS  = 26;
const int LO_MINUS = 25;

// Интервал между выборками: 10 мс = 100 Гц
const unsigned long SAMPLE_INTERVAL_MS = 10;
// ======================

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===== ВСТРОЕННАЯ HTML-СТРАНИЦА =====
static const char INDEX_HTML[] PROGMEM = R"=====(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ЭКГ Монитор</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
:root{
  --bg:#0d1117;--surf:#161b22;--surf2:#21262d;--brd:#30363d;
  --txt:#e6edf3;--muted:#8b949e;--pri:#58a6ff;
  --ok:#3fb950;--err:#f85149;--warn:#d29922;--ecg:#00d9ff;
}
*{box-sizing:border-box;margin:0;padding:0}
body{
  background:var(--bg);color:var(--txt);
  font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  height:100dvh;display:flex;flex-direction:column;overflow:hidden;
}
header{
  background:var(--surf);border-bottom:1px solid var(--brd);
  padding:10px 16px;display:flex;align-items:center;
  justify-content:space-between;flex-shrink:0;
}
.logo{display:flex;align-items:center;gap:8px;font-weight:700;font-size:15px}
.s-row{display:flex;align-items:center;gap:8px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--err);flex-shrink:0;transition:background .3s}
.dot.ok{background:var(--ok);animation:pulse 2s infinite}
.dot.warn{background:var(--warn)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
.s-txt{font-size:12px;color:var(--muted)}
.stats{
  display:flex;background:var(--surf2);border-bottom:1px solid var(--brd);flex-shrink:0;
}
.stat{flex:1;padding:8px 10px;border-right:1px solid var(--brd);text-align:center}
.stat:last-child{border-right:none}
.sl{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:.6px;margin-bottom:2px}
.sv{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums;letter-spacing:-.5px}
.sv.rec{color:var(--err)}
.chart-wrap{flex:1;padding:10px;min-height:0;position:relative}
#c{width:100%!important;height:100%!important;background:var(--surf);border-radius:8px;border:1px solid var(--brd)}
.lead-badge{
  display:none;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
  background:rgba(210,153,34,.15);border:1px solid var(--warn);color:var(--warn);
  padding:10px 20px;border-radius:8px;font-size:14px;font-weight:600;pointer-events:none;
  white-space:nowrap;
}
.lead-badge.show{display:block}
footer{
  background:var(--surf);border-top:1px solid var(--brd);
  padding:10px 14px;display:flex;gap:10px;flex-shrink:0;
}
.btn{
  padding:13px 0;border:none;border-radius:8px;font-size:14px;
  font-weight:700;cursor:pointer;transition:all .2s;flex:1;
}
.btn-rec{background:var(--ok);color:#000}
.btn-rec.recording{background:var(--err);color:#fff}
.btn-dl{background:var(--pri);color:#000;flex:0 0 auto;padding:13px 18px}
.btn-dl:disabled{opacity:.35;cursor:not-allowed}
</style>
</head>
<body>

<header>
  <div class="logo">
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#00d9ff" stroke-width="2.5">
      <polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/>
    </svg>
    ЭКГ Монитор
  </div>
  <div class="s-row">
    <div class="dot" id="dot"></div>
    <span class="s-txt" id="stxt">Подключение…</span>
  </div>
</header>

<div class="stats">
  <div class="stat">
    <div class="sl">Пульс уд/мин</div>
    <div class="sv" id="hr">—</div>
  </div>
  <div class="stat">
    <div class="sl">Частота Гц</div>
    <div class="sv" id="sps">—</div>
  </div>
  <div class="stat">
    <div class="sl">Запись</div>
    <div class="sv rec" id="dur">00:00</div>
  </div>
</div>

<div class="chart-wrap">
  <canvas id="c"></canvas>
  <div class="lead-badge" id="lb">⚠ Электрод не контактирует</div>
</div>

<footer>
  <button class="btn btn-rec" id="recBtn" onclick="toggleRec()">● Запись</button>
  <button class="btn btn-dl"  id="dlBtn"  onclick="dlCSV()" disabled>⬇ CSV</button>
</footer>

<script>
// ── Chart ────────────────────────────────────────────────
const POINTS = 600;
const buf = new Array(POINTS).fill(null);
const chart = new Chart(document.getElementById('c').getContext('2d'), {
  type: 'line',
  data: {
    labels: new Array(POINTS).fill(''),
    datasets: [{
      data: buf,
      borderColor: '#00d9ff',
      borderWidth: 1.5,
      pointRadius: 0,
      tension: 0,
      fill: false,
      spanGaps: false,
    }]
  },
  options: {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    plugins: { legend: { display: false }, tooltip: { enabled: false } },
    scales: {
      x: { display: false },
      y: {
        min: 0, max: 4095,
        ticks: { color: '#8b949e', font: { size: 9 }, maxTicksLimit: 6 },
        grid: { color: 'rgba(48,54,61,0.6)' },
        border: { color: '#30363d' },
      }
    }
  }
});

// ── HR Detection (crossing threshold) ───────────────────
let hrBuf = [], lastPeakTs = 0, lastVal = 0, prevVal = 0;
const HR_THRESHOLD = 2800;
const HR_REFRACTORY = 350; // мс между пиками

function detectHR(val, ts) {
  // R-peak: пересечение порога снизу вверх
  if (prevVal < HR_THRESHOLD && val >= HR_THRESHOLD) {
    if (lastPeakTs > 0) {
      const interval = ts - lastPeakTs;
      if (interval > HR_REFRACTORY) {
        const bpm = Math.round(60000 / interval);
        if (bpm > 25 && bpm < 220) {
          hrBuf.push(bpm);
          if (hrBuf.length > 6) hrBuf.shift();
          const avg = Math.round(hrBuf.reduce((a, b) => a + b) / hrBuf.length);
          document.getElementById('hr').textContent = avg;
        }
      }
    }
    lastPeakTs = ts;
  }
  prevVal = val;
}

// ── WebSocket ────────────────────────────────────────────
let ws, recording = false, recData = [], recStart = 0, recTimer = null, sps = 0;

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');

  ws.onopen = () => {
    document.getElementById('dot').className = 'dot ok';
    document.getElementById('stxt').textContent = 'Подключено';
  };
  ws.onclose = () => {
    document.getElementById('dot').className = 'dot';
    document.getElementById('stxt').textContent = 'Переподключение…';
    setTimeout(connect, 2000);
  };
  ws.onerror = () => ws.close();

  ws.onmessage = e => {
    const p = e.data.split(',');
    if (p.length < 3) return;
    const ts = parseInt(p[0]);
    const val = parseInt(p[1]);
    const lo = p[2] === '1';
    sps++;

    document.getElementById('lb').className = 'lead-badge' + (lo ? ' show' : '');

    if (lo) {
      document.getElementById('dot').className = 'dot warn';
      document.getElementById('stxt').textContent = 'Электрод не контактирует';
      buf.shift(); buf.push(null);
    } else {
      document.getElementById('dot').className = 'dot ok';
      document.getElementById('stxt').textContent = 'Подключено';
      buf.shift(); buf.push(val);
      detectHR(val, ts);
      if (recording) {
        recData.push(new Date().toISOString() + ',' + ts + ',' + val);
      }
    }
    chart.update('none');
  };
}

// Счётчик частоты выборок
setInterval(() => {
  document.getElementById('sps').textContent = sps || '—';
  sps = 0;
}, 1000);

// ── Запись ───────────────────────────────────────────────
function toggleRec() {
  const btn = document.getElementById('recBtn');
  const dl  = document.getElementById('dlBtn');
  if (!recording) {
    recording = true; recData = []; recStart = Date.now();
    btn.textContent = '■ Стоп'; btn.classList.add('recording');
    dl.disabled = true;
    recTimer = setInterval(() => {
      const s = Math.floor((Date.now() - recStart) / 1000);
      document.getElementById('dur').textContent =
        String(Math.floor(s / 60)).padStart(2, '0') + ':' +
        String(s % 60).padStart(2, '0');
    }, 500);
  } else {
    recording = false; clearInterval(recTimer);
    btn.textContent = '● Запись'; btn.classList.remove('recording');
    dl.disabled = recData.length === 0;
  }
}

// ── Скачать CSV ──────────────────────────────────────────
function dlCSV() {
  const d = new Date();
  const pad = n => String(n).padStart(2, '0');
  const name = 'ECG_' +
    d.getFullYear() + pad(d.getMonth()+1) + pad(d.getDate()) + '_' +
    pad(d.getHours()) + pad(d.getMinutes()) + pad(d.getSeconds()) + '.csv';
  const content = 'datetime_iso,esp32_uptime_ms,adc_raw\n' + recData.join('\n');
  const blob = new Blob([content], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  URL.revokeObjectURL(a.href);
}

connect();
</script>
</body>
</html>
)=====";
// ====================================

void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Клиент #%u подключился (%s)\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Клиент #%u отключился\n", client->id());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ЭКГ Монитор (WiFi) ===");

  pinMode(LO_PLUS,  INPUT);
  pinMode(LO_MINUS, INPUT);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("Точка доступа: %s\n", AP_SSID);
  Serial.printf("IP-адрес: %s\n", WiFi.softAPIP().toString().c_str());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", INDEX_HTML);
  });

  server.begin();
  Serial.println("Сервер запущен. Откройте в браузере: http://192.168.4.1");
}

void loop() {
  static unsigned long lastSample = 0;
  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    // Периодически чистим отключившихся клиентов
    ws.cleanupClients();

    if (ws.count() > 0) {
      bool leadOff = (digitalRead(LO_PLUS)  == HIGH) ||
                     (digitalRead(LO_MINUS) == HIGH);
      int val = leadOff ? 0 : analogRead(ECG_PIN);

      // Формат сообщения: "uptime_ms,adc_value,lead_off_flag"
      char msg[48];
      snprintf(msg, sizeof(msg), "%lu,%d,%d", now, val, leadOff ? 1 : 0);
      ws.textAll(msg);
    }
  }
}
