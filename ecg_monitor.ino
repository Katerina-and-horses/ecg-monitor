/*
 * ============================================================
 *  ECG MONITOR — ESP32 + AD8232 + SD-карта
 *  WiFi AP + WebSocket (стрим + командный протокол) + PWA
 *  Поддержка людей и лошадей (base-apex отведение для лошади)
 *  v3.0 — SD как источник истины, метки, профили, captive portal
 * ============================================================
 *
 *  ПОДКЛЮЧЕНИЕ AD8232 → ESP32 (DevKit WROOM-32):
 *    OUTPUT  → GPIO34  (только ADC1, не через Flash-пины!)
 *    LO+     → GPIO26
 *    LO-     → GPIO25
 *    SDN     → не подключён (AD8232 всегда активен)
 *    3V3     → 3.3V
 *    GND     → GND
 *
 *  ПОДКЛЮЧЕНИЕ SD-МОДУЛЯ (обычный SPI microSD-брейкаут):
 *    CS      → GPIO5
 *    MOSI    → GPIO23
 *    MISO    → GPIO19
 *    SCK     → GPIO18
 *    VCC     → 3V3 (или 5V, если на модуле есть свой регулятор — см. маркировку платы)
 *    GND     → GND
 *
 *  ЭЛЕКТРОДЫ (см. ЭКГ_лошадь_расположение_электродов.pdf в этой папке):
 *    🔴 = IN− (негатив) · 🟡 = IN+ (позитив) · 🟢 = RLD, активный
 *    гаситель синфазной помехи — НЕ земля, в измерение не входит и вектор
 *    отведения не задаёт, но контакт критичен не меньше сигнальных
 *    (при сухом 🟢 гаситель не работает и 50 Гц заливает сигнал).
 *    Смачивать под каждым кольцом (физраствор / вода / 70% спирт / гель),
 *    в т.ч. под 🟢 — локально, не всю окружность ремня. 🟢 НЕ соединять
 *    с GND платы / сетевой землёй; отнести от сигнальной пары ≥ ширины ладони.
 *
 *    Человек (IEC): 🔴 RA (правая рука) · 🟡 LA (левая рука) · 🟢 RL (правая нога)
 *    Лошадь — 4 варианта 3-электродных отведений (все в PDF):
 *      В2 (ОСНОВНОЙ, модиф. base-apex, всё на ЛЕВОМ боку, раскладка Polar Equine):
 *        🔴 → левая сторона холки / вдоль остистых отростков
 *        🟡 → левая грудная стенка сразу за локтем (зона верхушечного толчка)
 *        🟢 → верх плеча слева (над локтем, ближе к лопатке) или мечевидный отросток
 *      В1 поперечное (локоть↔локоть) — второй вектор для экстрасистол
 *      В3 классическое base-apex (правый ярёмный жёлоб ↔ левый локоть) — эталон морфологии
 *      В4 диагональ (основание шеи слева ↔ грудина/правый локоть) — компромисс, 1 ремень
 *    Полоса тракта ~0.5–40 Гц: P и T проходят, сегмент ST и точный QT — недостоверны.
 *    Усиление AD8232 фиксировано (~×1100), правится только слайдером «УСИЛ» в UI.
 *
 *  АРХИТЕКТУРА (почему так):
 *   - Запись сессии пишется на SD НЕПРЕРЫВНО и НЕЗАВИСИМО от того,
 *     жив ли WebSocket / жива ли вкладка браузера на телефоне —
 *     это чинит проблему "линия теряется, данные перестают
 *     передаваться" из предыдущих версий, где вся запись жила
 *     только в JS-памяти браузера.
 *   - SD-запись выполняется в отдельной FreeRTOS-задаче на ядре 0,
 *     чтобы не блокировать сэмплирование/трансляцию на ядре 1.
 *   - Сетевой стек ОСТАЁТСЯ синхронным (WebServer + WebSocketsServer) —
 *     в v1 пробовали ESPAsyncWebServer/AsyncTCP и осознанно
 *     отказались от него как от менее стабильного. Здесь его
 *     не трогаем.
 *   - Восстановлен DNS/captive-portal обработчик из v3
 *     (ecg_wifi_monitor_v3.ino) — по комментариям той версии,
 *     Android блокирует/душит WebSocket на закрытой AP-сети без
 *     интернета через механизм проверки captive portal. Это было
 *     решено в v3 и потеряно в текущей (на момент ревизии) версии —
 *     возможная вторая причина обрывов связи.
 *   - Удаления записей с SD НЕТ — сознательное решение: SD хранит
 *     всё постоянно, телефон только выгружает копии для просмотра.
 *
 *  ЗАВИСИМОСТИ (Library Manager):
 *    - arduinoWebSockets by Markus Sattler (v2.x)
 *    - ArduinoJson by Benoit Blanchon (v7.x)
 *    (WiFi, WebServer, DNSServer, SD, SPI, FS — входят в ESP32 core)
 *
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <string.h>

// ============================================================
//  КОНФИГУРАЦИЯ
// ============================================================

#define ECG_PIN      34
#define LO_PLUS      26
#define LO_MINUS     25

#define SD_CS        5
#define SD_SCK       18
#define SD_MISO      19
#define SD_MOSI      23

#define SAMPLE_RATE  250
#define BATCH_SIZE   10          // сэмплов в одном live-пакете WS
#define RING_CAP     2000        // ~8 сек запаса на случай задержки SD (250Гц*8с)
#define SD_FLUSH_MS  1000        // как часто сбрасываем кольцевой буфер на SD
#define MAX_MARKS    60
#define MARK_LABEL_LEN 32

const char* AP_SSID = "ECG_Monitor";
const char* AP_PASS = "ecg12345";

const char* REC_DIR      = "/rec";
const char* INDEX_PATH   = "/index.json";
const char* PROFILES_PATH = "/profiles.json";

// ============================================================
//  ОБЪЕКТЫ
// ============================================================

WebServer        httpServer(80);
WebSocketsServer wsServer(81);
DNSServer         dnsServer;
IPAddress         apIP;

// Мьютекс, охраняющий ЛЮБОЙ доступ к SD-карте (файлы .dat, index.json,
// profiles.json) — SD-карта не терпит параллельного доступа с двух ядер.
SemaphoreHandle_t sdMux;

// ============================================================
//  БУФЕР ВЫБОРОК ДЛЯ LIVE-ТРАНСЛЯЦИИ (как раньше)
// ============================================================

volatile uint16_t adcBuf[BATCH_SIZE];
volatile uint8_t  adcIdx   = 0;
volatile bool     pktReady = false;

portMUX_TYPE timerMux  = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t*  sampTimer = NULL;
volatile bool sampleTick = false;
void IRAM_ATTR onTimer() { sampleTick = true; }

// ============================================================
//  КОЛЬЦЕВОЙ БУФЕР ДЛЯ ЗАПИСИ НА SD
//  Формат сэмпла: бит15 = lead-off, биты0-11 = ADC (0..4095)
// ============================================================

volatile uint16_t ring[RING_CAP];
volatile uint32_t ringHead = 0;   // пишет loop() (ядро 1)
volatile uint32_t ringTail = 0;   // читает sdTask() (ядро 0)
SemaphoreHandle_t  ringMux;

// ============================================================
//  СОСТОЯНИЕ ЗАПИСИ
// ============================================================

volatile bool recording = false;
File sdFile;
String  curSessionId, curProfileId, curMode;
volatile uint32_t sessionSampleCount = 0;
double  sessionStartEpochMs = 0;

struct Mark { uint32_t sampleIdx; char label[MARK_LABEL_LEN]; };
Mark    marks[MAX_MARKS];
uint8_t marksCount = 0;

// "Мягкие часы": телефон присылает своё время один раз при коннекте,
// дальше epoch-время считаем как offset + millis(). Не используется
// для тайминга ударов сердца (для этого — номер сэмпла), только для
// человекочитаемых меток времени/имён файлов.
double phoneEpochOffsetMs = 0;   // = phoneNowMs - millis() в момент SET_TIME
bool   timeSynced = false;

double nowEpochMs() {
  return timeSynced ? (phoneEpochOffsetMs + (double)millis()) : (double)millis();
}

// ============================================================
//  ФОРВАРД-ДЕКЛАРАЦИИ
// ============================================================

void handleWsText(uint8_t num, uint8_t* payload, size_t len);
void sdTask(void* param);
void handleDownload(String id);
void handleCaptive();
void handleNotFound();

// ============================================================
//  SD: ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (все под sdMux)
// ============================================================

bool sdEnsureFile(const char* path, const char* emptyContent) {
  xSemaphoreTake(sdMux, portMAX_DELAY);
  bool exists = SD.exists(path);
  if (!exists) {
    File f = SD.open(path, FILE_WRITE);
    if (f) { f.print(emptyContent); f.close(); exists = true; }
  }
  xSemaphoreGive(sdMux);
  return exists;
}

bool sdReadJson(const char* path, JsonDocument& doc) {
  xSemaphoreTake(sdMux, portMAX_DELAY);
  File f = SD.open(path, FILE_READ);
  bool ok = false;
  if (f) {
    DeserializationError err = deserializeJson(doc, f);
    ok = !err;
    f.close();
  }
  xSemaphoreGive(sdMux);
  return ok;
}

bool sdWriteJson(const char* path, JsonDocument& doc) {
  xSemaphoreTake(sdMux, portMAX_DELAY);
  SD.remove(path);   // FILE_WRITE на ESP32 дописывает в конец, а не перезаписывает —
                      // поэтому сперва удаляем старый файл
  File f = SD.open(path, FILE_WRITE);
  bool ok = false;
  if (f) {
    ok = serializeJson(doc, f) > 0;
    f.close();
  }
  xSemaphoreGive(sdMux);
  return ok;
}

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

  #header {
    display: flex; align-items: center; justify-content: space-between;
    padding: 8px 14px; background: var(--bg2); border-bottom: 1px solid var(--border);
    height: 52px; flex-shrink: 0;
  }
  #appTitle { font-family: var(--title); font-size: 14px; font-weight: 700; color: var(--green); letter-spacing: 3px; text-shadow: 0 0 12px var(--green); }
  #appSub   { font-size: 9px; color: var(--dim); letter-spacing: 1px; margin-top: 2px; }

  #leadBlock { display: flex; align-items: center; gap: 8px; }
  #leadDot { width: 10px; height: 10px; border-radius: 50%; background: var(--red); box-shadow: 0 0 10px var(--red); transition: background 0.3s, box-shadow 0.3s; flex-shrink: 0; }
  #leadDot.ok   { background: var(--green); box-shadow: 0 0 10px var(--green); animation: breathe 2s ease-in-out infinite; }
  #leadDot.conn { background: var(--amber); box-shadow: 0 0 10px var(--amber); }
  @keyframes breathe { 0%,100%{opacity:1} 50%{opacity:0.5} }
  #leadText { font-size: 10px; color: var(--dim); letter-spacing: 1px; max-width: 90px; line-height: 1.3; }

  #bpmBlock { text-align: right; }
  #bpmVal   { font-family: var(--title); font-size: 26px; color: var(--cyan); text-shadow: 0 0 16px var(--cyan); line-height: 1; letter-spacing: 2px; }
  #bpmUnit  { font-size: 9px; color: var(--dim); letter-spacing: 2px; }
  #bpmVal.pulse { animation: bpmFlash 0.15s ease-out; }
  @keyframes bpmFlash { 0%{color:#fff;text-shadow:0 0 30px #fff} 100%{color:var(--cyan);text-shadow:0 0 16px var(--cyan)} }

  #tabBar { display: flex; background: var(--bg2); border-bottom: 1px solid var(--border); flex-shrink: 0; }
  .tab { flex: 1; padding: 9px 4px; background: transparent; border: none; border-bottom: 2px solid transparent; color: var(--dim); font-family: var(--font); font-size: 11px; letter-spacing: 2px; cursor: pointer; transition: color 0.2s, border-color 0.2s; -webkit-appearance: none; }
  .tab.active { color: var(--green); border-bottom-color: var(--green); }
  #libCount { font-size: 9px; color: var(--amber); margin-left: 3px; }

  #monitorSection { flex: 1; display: flex; flex-direction: column; min-height: 0; }
  #ecgWrap { position: relative; background: #000; flex: 1; overflow: hidden; min-height: 0; }
  #ecgCanvas { display: block; width: 100%; height: 100%; image-rendering: pixelated; }
  #overlay { position: absolute; top: 8px; left: 10px; font-size: 10px; color: #2a4a2a; letter-spacing: 1px; pointer-events: none; line-height: 1.6; }
  #recFlag { position:absolute; top:8px; right:10px; font-size:10px; color:var(--red); letter-spacing:1px; display:none; }
  #recFlag.on { display:block; animation: recPulse 1.2s infinite; }

  #controls { display: flex; align-items: center; gap: 8px; padding: 8px 12px; background: var(--bg2); border-top: 1px solid var(--border); flex-shrink: 0; flex-wrap: wrap; min-height: 52px; }

  .btn { padding: 7px 12px; border: 1px solid currentColor; border-radius: 4px; background: transparent; font-family: var(--font); font-size: 11px; letter-spacing: 1px; cursor: pointer; transition: background 0.15s, box-shadow 0.15s; white-space: nowrap; -webkit-appearance: none; }
  .btn:active { filter: brightness(1.4); }
  .btn:disabled { opacity: 0.25; pointer-events: none; }

  #recBtn  { color: var(--red); }
  #recBtn.rec { background: rgba(255,59,59,0.15); box-shadow: 0 0 8px var(--red); animation: recPulse 1.2s infinite; }
  @keyframes recPulse { 0%,100%{box-shadow:0 0 8px var(--red)} 50%{box-shadow:0 0 20px var(--red)} }
  #markBtn { color: var(--amber); display: none; }
  #markBtn.show { display: inline-block; }
  #modeBtn  { color: var(--cyan); }
  #modeBtn.horse { background: rgba(0,229,255,0.1); }

  #profileGroup { display: flex; align-items: center; gap: 6px; margin-left: auto; }
  #profileSelect { background: #000; border: 1px solid var(--border); color: var(--text); font-family: var(--font); font-size: 11px; padding: 6px 8px; border-radius: 4px; max-width: 120px; }
  #gainGroup { display: flex; align-items: center; gap: 6px; }
  #gainGroup label { font-size: 10px; color: var(--dim); letter-spacing: 1px; }
  #gainRange { -webkit-appearance: none; width: 70px; height: 3px; border-radius: 2px; background: var(--border); outline: none; cursor: pointer; }
  #gainRange::-webkit-slider-thumb { -webkit-appearance: none; width: 14px; height: 14px; border-radius: 50%; background: var(--green); box-shadow: 0 0 6px var(--green); }
  #gainLabel { font-size: 11px; color: var(--green); min-width: 24px; }

  #filesSection { flex: 1; display: none; flex-direction: column; min-height: 0; overflow: hidden; }
  #libScroll { flex: 1; overflow-y: auto; }
  .profGroup { border-bottom: 1px solid var(--border); }
  .profHead { display:flex; align-items:center; gap:8px; padding: 9px 12px; background: var(--bg2); position: sticky; top:0; }
  .profHead .pname { font-family: var(--title); font-size: 11px; color: var(--green); letter-spacing: 2px; flex:1; }
  .profHead .pcount { font-size: 9px; color: var(--dim); }

  .fileRow { display: flex; align-items: center; gap: 8px; padding: 9px 12px; border-bottom: 1px solid var(--border); cursor: pointer; transition: background 0.15s; border-left: 3px solid transparent; }
  .fileRow:active { background: rgba(0,255,136,0.04); }
  .fileRow.sel0 { border-left-color: var(--t0); }
  .fileRow.sel1 { border-left-color: var(--t1); }
  .fileRow.sel2 { border-left-color: var(--t2); }
  .fName { font-size: 12px; color: var(--text); flex: 1; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .fMeta { font-size: 9px; color: var(--dim); line-height: 1.6; text-align: right; flex-shrink: 0; min-width: 70px; }
  .fActs { display: flex; gap: 2px; flex-shrink: 0; }
  .iBtn  { background: transparent; border: none; color: var(--dim); font-size: 14px; cursor: pointer; padding: 3px 5px; -webkit-appearance: none; line-height: 1; border-radius: 3px; }
  .iBtn:active { background: rgba(255,255,255,0.07); color: var(--text); }
  .iBtn.cached { color: var(--green); }
  .emptyMsg { padding: 36px 20px; text-align: center; color: var(--dim); font-size: 11px; line-height: 2.2; letter-spacing: 1px; }

  #analysisPane { flex-shrink: 0; display: flex; flex-direction: column; min-height:0; max-height: 46%; border-top: 1px solid var(--border); }
  #compWrap { flex: 1; background: #000; position: relative; min-height: 90px; overflow: hidden; }
  #compCanvas { display: block; width: 100%; height: 100%; }
  #compPlaceholder { position: absolute; inset: 0; display: flex; align-items: center; justify-content: center; color: var(--dim); font-size: 11px; letter-spacing: 2px; flex-direction: column; gap: 8px; text-align:center; padding: 0 20px; }
  #compPlaceholder span { font-size: 24px; }
  #statsPane { flex-shrink: 0; display: flex; overflow-x: auto; background: var(--bg2); border-top: 1px solid var(--border); }
  .statCard { flex: 1; min-width: 100px; padding: 8px 10px; border-right: 1px solid var(--border); }
  .statCard:last-child { border-right: none; }
  .stTitle { font-size: 8px; color: var(--dim); letter-spacing: 1px; margin-bottom: 4px; text-transform: uppercase; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  .stVal   { font-family: var(--title); font-size: 16px; line-height: 1.1; }
  .stSub   { font-size: 9px; color: var(--dim); margin-top: 2px; }
  .stWarn  { font-size: 9px; color: var(--amber); margin-top: 3px; line-height:1.4; }

  #modal { position: fixed; inset: 0; background: rgba(0,0,0,0.88); display: none; align-items: center; justify-content: center; z-index: 200; padding: 20px; }
  #modalBox { background: var(--bg2); border: 1px solid var(--border); border-radius: 10px; padding: 22px 20px; width: 100%; max-width: 320px; }
  #modalBox h3 { font-family: var(--title); font-size: 11px; color: var(--green); letter-spacing: 3px; margin-bottom: 16px; text-shadow: 0 0 8px var(--green); }
  .fRow { margin-bottom: 12px; }
  .fRow label { display: block; font-size: 10px; color: var(--dim); letter-spacing: 1px; margin-bottom: 5px; }
  .fRow input[type=text], .fRow select { width: 100%; background: #000; border: 1px solid var(--border); border-radius: 4px; padding: 9px 10px; color: var(--text); font-family: var(--font); font-size: 13px; outline: none; -webkit-appearance: none; }
  .fRow input[type=text]:focus { border-color: var(--green); }
  #modalInfo { font-size: 10px; color: var(--dim); line-height: 1.9; margin-bottom: 14px; padding: 8px 10px; background: #000; border-radius: 4px; border: 1px solid var(--border); }
  .mBtns { display: flex; gap: 8px; }
  .mBtns .btn { flex: 1; text-align: center; padding: 9px 6px; font-size: 10px; }
  .btn-ghost   { color: var(--dim); }
  .btn-primary { color: #000 !important; background: var(--green); border-color: var(--green); font-weight: bold; }

  #splash { position: fixed; inset: 0; background: rgba(0,0,0,0.93); display: flex; align-items: center; justify-content: center; z-index: 100; padding: 20px; }
  #splashBox { background: var(--bg2); border: 1px solid var(--border); border-radius: 12px; padding: 28px 24px; max-width: 340px; width: 100%; }
  #splashBox h2 { font-family: var(--title); color: var(--green); font-size: 18px; letter-spacing: 4px; margin-bottom: 16px; text-align: center; text-shadow: 0 0 12px var(--green); }
  .splashRow { display: flex; align-items: center; gap: 10px; margin: 8px 0; font-size: 13px; color: var(--text); }
  .splashRow .dot { width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0; }
  .dot-r { background: var(--red); } .dot-y { background: #ffdd00; } .dot-g { background: var(--green); }
  .splashDiv { border: none; border-top: 1px solid var(--border); margin: 14px 0; }
  .splashInfo { font-size: 12px; color: var(--dim); line-height: 1.8; margin-bottom: 6px; }
  .splashInfo b { color: var(--cyan); }
  #splashBtn { display: block; width: 100%; margin-top: 18px; padding: 11px; background: var(--green); color: #000; border: none; border-radius: 6px; font-family: var(--title); font-size: 13px; letter-spacing: 2px; cursor: pointer; font-weight: 700; -webkit-appearance: none; }
</style>
</head>
<body>

<div id="header">
  <div>
    <div id="appTitle">ECG MONITOR</div>
    <div id="appSub">ESP32 · AD8232 · SD · 250 HZ</div>
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

<div id="tabBar">
  <button class="tab active" id="tabMonitor" onclick="switchTab('monitor')">⬤ МОНИТОР</button>
  <button class="tab"        id="tabFiles"   onclick="switchTab('files')">📁 БИБЛИОТЕКА<span id="libCount"></span></button>
</div>

<div id="monitorSection">
  <div id="ecgWrap">
    <canvas id="ecgCanvas"></canvas>
    <div id="overlay">25 MM/S · 10 MM/MV</div>
    <div id="recFlag">● ЗАПИСЬ ИДЁТ (на SD)</div>
  </div>
  <div id="controls">
    <button class="btn" id="recBtn"  onclick="toggleRec()">⬤ ЗАПИСЬ</button>
    <button class="btn" id="markBtn" onclick="sendMark()">⚑ МЕТКА</button>
    <button class="btn" id="modeBtn" onclick="toggleMode()">🐴 ЛОШАДЬ</button>
    <div id="gainGroup">
      <label>УСИЛ</label>
      <input type="range" id="gainRange" min="1" max="20" value="5"
             oninput="document.getElementById('gainLabel').textContent=this.value+'×'; gain=+this.value;">
      <span id="gainLabel">5×</span>
    </div>
    <div id="profileGroup">
      <select id="profileSelect" onchange="onProfileChange()"></select>
    </div>
  </div>
</div>

<div id="filesSection">
  <div id="libScroll"><div id="libList"><div class="emptyMsg">библиотека пуста</div></div></div>
  <div id="analysisPane">
    <div id="compWrap">
      <canvas id="compCanvas"></canvas>
      <div id="compPlaceholder"><span>📈</span>Выгрузите запись на телефон и откройте её для анализа</div>
    </div>
    <div id="statsPane"></div>
  </div>
</div>

<div id="modal">
  <div id="modalBox">
    <h3 id="modalTitle">—</h3>
    <div id="modalInfo">—</div>
    <div class="fRow" id="modalNameRow" style="display:none">
      <label>Метка (необязательно)</label>
      <input type="text" id="modalInput" placeholder="испугалась / легла / шум рядом">
    </div>
    <div class="mBtns">
      <button class="btn btn-ghost"   onclick="hideModal()">✕ ОТМЕНА</button>
      <button class="btn btn-primary" id="modalOkBtn" onclick="modalOk()">OK</button>
    </div>
  </div>
</div>

<div id="splash">
  <div id="splashBox">
    <h2>ECG MONITOR</h2>
    <div class="splashRow"><div class="dot dot-r"></div><span>Красный (IN−): человек — RA / лошадь — левая сторона холки</span></div>
    <div class="splashRow"><div class="dot dot-y"></div><span>Жёлтый (IN+): человек — LA / лошадь — левая грудная стенка за локтем</span></div>
    <div class="splashRow"><div class="dot dot-g"></div><span>Зелёный (RLD, не земля): человек — RL / лошадь — верх плеча слева</span></div>
    <hr class="splashDiv">
    <div class="splashInfo">Лошадь: основное отведение — модиф. base-apex, всё на левом боку (раскладка Polar Equine). Другие 3 варианта — в инструкции <b>ЭКГ_лошадь_расположение_электродов.pdf</b>.</div>
    <div class="splashInfo">Смачивать кожу под <b>каждым</b> кольцом, включая зелёное — иначе ложная «потеря линии» (lead-off) и наводка 50 Гц.</div>
    <div class="splashInfo">WiFi: <b>ECG_Monitor</b> · Пароль: <b>ecg12345</b><br>Адрес: <b>192.168.4.1</b></div>
    <div class="splashInfo">Запись пишется на SD-карту устройства и не зависит от связи. Телефон — пульт управления и библиотека для просмотра.</div>
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
const HR_RANGE = { human: [30, 220], horse: [20, 100] };
const REFRACT  = { human: 350, horse: 700 };

// ====================================================
//  СОСТОЯНИЕ
// ====================================================
let canvas, ctx, W, H;
let horseMode = false;
let gain = 5;
let recording = false;
let ws, wsReady = false;
let pxPerSample, cursorX = 0, prevY = null;
let peaks = [], lastPeakMs = 0, bpm = 0;
let baseline = 2048;
const ALPHA_BL = 0.002;
let peakSmooth = 0, peakMax = 0;

let profiles = [];         // [{id,name,species}]
let sessions = [];         // из LIST
let selectedIds = [];      // до 3 id для сравнения (из кэша телефона)
let pendingCb = null;      // callback модалки

// ====================================================
//  IndexedDB — АРХИВ ТЕЛЕФОНА (копии выгруженных записей)
// ====================================================
const DB_NAME = 'ECG_DB', DB_VER = 1, STORE = 'recordings';
let db;

function openDB() {
  return new Promise((res, rej) => {
    const req = indexedDB.open(DB_NAME, DB_VER);
    req.onupgradeneeded = e => {
      const d = e.target.result;
      if (!d.objectStoreNames.contains(STORE))
        d.createObjectStore(STORE, { keyPath: 'id' });
    };
    req.onsuccess = e => { db = e.target.result; res(db); };
    req.onerror   = rej;
  });
}
function dbStore(mode) { return db.transaction(STORE, mode).objectStore(STORE); }
function dbGetAll() { return new Promise((res, rej) => { const r = dbStore('readonly').getAll(); r.onsuccess = () => res(r.result); r.onerror = rej; }); }
function dbGet(id)   { return new Promise((res, rej) => { const r = dbStore('readonly').get(id);   r.onsuccess = () => res(r.result); r.onerror = rej; }); }
function dbPut(rec)  { return new Promise((res, rej) => { const r = dbStore('readwrite').put(rec); r.onsuccess = () => res(r.result); r.onerror = rej; }); }

// ====================================================
//  ЗАПУСК
// ====================================================
async function startApp() {
  document.getElementById('splash').style.display = 'none';
  await openDB();
  initCanvas();
  connectWS();
}

function switchTab(tab) {
  const mon = tab === 'monitor';
  document.getElementById('monitorSection').style.display = mon ? 'flex' : 'none';
  document.getElementById('filesSection').style.display   = mon ? 'none' : 'flex';
  document.getElementById('tabMonitor').classList.toggle('active',  mon);
  document.getElementById('tabFiles').classList.toggle('active', !mon);
  if (!mon) { wsSend({cmd:'LIST'}); wsSend({cmd:'LIST_PROFILES'}); }
}

// ====================================================
//  КАНВАС МОНИТОРА (без изменений в принципе работы)
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
  canvas.width = W; canvas.height = H;
  pxPerSample = PX_PER_S / FS;
  clearCanvas(); cursorX = 0; prevY = null;
}
function clearCanvas() { ctx.fillStyle = '#000'; ctx.fillRect(0,0,W,H); drawGrid(ctx,W,H); }
function drawGrid(c, w, h) {
  const big = PX_PER_S * 0.2, sm = big / 5;
  c.lineWidth = 0.5;
  for (let x = 0; x < w; x += sm) { c.strokeStyle = (Math.abs(x - Math.round(x/big)*big) < 0.5) ? '#1c2e1c' : '#0e180e'; c.beginPath(); c.moveTo(x,0); c.lineTo(x,h); c.stroke(); }
  for (let y = 0; y < h; y += sm) { c.strokeStyle = (Math.abs(y - Math.round(y/big)*big) < 0.5) ? '#1c2e1c' : '#0e180e'; c.beginPath(); c.moveTo(0,y); c.lineTo(w,y); c.stroke(); }
  c.strokeStyle = '#0f200f'; c.lineWidth = 1; c.beginPath(); c.moveTo(0,h/2); c.lineTo(w,h/2); c.stroke();
}
function redrawGridStrip(x0, sw) {
  const big = PX_PER_S * 0.2, sm = big / 5;
  ctx.lineWidth = 0.5;
  for (let x = Math.floor(x0/sm)*sm; x <= x0+sw; x += sm) { if (x < x0) continue; ctx.strokeStyle = (Math.abs(x - Math.round(x/big)*big) < 1) ? '#1c2e1c' : '#0e180e'; ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,H); ctx.stroke(); }
  for (let y = 0; y < H; y += sm) { ctx.strokeStyle = (Math.abs(y - Math.round(y/big)*big) < 1) ? '#1c2e1c' : '#0e180e'; ctx.beginPath(); ctx.moveTo(x0,y); ctx.lineTo(x0+sw,y); ctx.stroke(); }
  ctx.strokeStyle = '#0f200f'; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x0,H/2); ctx.lineTo(x0+sw,H/2); ctx.stroke();
}

// ====================================================
//  WEBSOCKET — стрим (бинарные фреймы) + команды (текст)
// ====================================================
function connectWS() {
  setLead('conn', 'подключение...');
  ws = new WebSocket(`ws://${location.hostname}:${WS_PORT}`);
  ws.binaryType = 'arraybuffer';
  ws.onopen  = () => { wsReady = true; setLead('conn', 'ожидание данных'); wsSend({cmd:'SET_TIME', t: Date.now()}); wsSend({cmd:'LIST_PROFILES'}); };
  ws.onclose = () => { wsReady = false; setLead('off', 'нет связи'); setTimeout(connectWS, 2000); };
  ws.onerror = () => ws.close();
  ws.onmessage = evt => {
    if (typeof evt.data === 'string') { onWsCommand(evt.data); return; }
    const dv  = new DataView(evt.data);
    const n   = (evt.data.byteLength - 1) / 2;
    const lo  = !!(dv.getUint8(evt.data.byteLength - 1) & 0x01);
    setLead(lo ? 'off' : 'ok', lo ? 'нет контакта' : 'сигнал OK');
    for (let i = 0; i < n; i++) processSample(dv.getUint16(i * 2, true), lo);
  };
}
function wsSend(obj) { if (wsReady) ws.send(JSON.stringify(obj)); }

function onWsCommand(text) {
  let msg; try { msg = JSON.parse(text); } catch(e) { return; }
  if (msg.type === 'list') { sessions = msg.sessions || []; renderLibrary(); }
  else if (msg.type === 'profiles') { profiles = msg.profiles || []; renderProfileSelect(); renderLibrary(); }
  else if (msg.type === 'rec_started') { recording = true; onRecStateChanged(); }
  else if (msg.type === 'rec_stopped') { recording = false; onRecStateChanged(); wsSend({cmd:'LIST'}); }
  else if (msg.type === 'error') { alert('Ошибка устройства: ' + msg.msg); }
}

// ====================================================
//  ОБРАБОТКА ВЫБОРКИ / ОТРИСОВКА
// ====================================================
function processSample(raw, leadOff) {
  baseline += ALPHA_BL * (raw - baseline);
  const v = (raw - baseline) / 2048.0;
  if (!leadOff) detectPeak(v);
  drawSample(v, leadOff);
}
function drawSample(v, leadOff) {
  const y = H/2 - v * (H * 0.4) * gain;
  const x = cursorX;
  ctx.fillStyle = '#000'; ctx.fillRect(x + 1, 0, ERASE_W, H);
  redrawGridStrip(x + 1, ERASE_W);
  if (prevY !== null && !leadOff) {
    ctx.beginPath(); ctx.strokeStyle = '#00ff66'; ctx.lineWidth = 1.8;
    ctx.shadowColor = '#00ff66'; ctx.shadowBlur = 5;
    ctx.moveTo(x - pxPerSample, prevY); ctx.lineTo(x, y); ctx.stroke(); ctx.shadowBlur = 0;
  } else if (leadOff && prevY !== null) {
    ctx.beginPath(); ctx.strokeStyle = '#2a2a2a'; ctx.lineWidth = 1; ctx.setLineDash([3,5]);
    ctx.moveTo(x - pxPerSample, prevY); ctx.lineTo(x, H/2); ctx.stroke(); ctx.setLineDash([]);
  }
  prevY = leadOff ? null : y;
  cursorX += pxPerSample;
  if (cursorX >= W) { cursorX = 0; prevY = null; }
}
function detectPeak(v) {
  peakSmooth = peakSmooth * 0.97 + v * 0.03;
  peakMax    = Math.max(peakMax * 0.9995, Math.abs(v));
  const thr    = Math.max(0.15, peakMax * 0.6);
  const refMs  = horseMode ? REFRACT.horse : REFRACT.human;
  const now    = Date.now();
  if (v > thr && (now - lastPeakMs) > refMs) {
    lastPeakMs = now; peaks.push(now);
    peaks = peaks.filter(t => now - t < 10000);
    if (peaks.length >= 3) {
      const ivs = peaks.slice(1).map((t,i) => t - peaks[i]);
      const avg = ivs.reduce((a,b)=>a+b,0) / ivs.length;
      const nb  = Math.round(60000 / avg);
      const rng = horseMode ? HR_RANGE.horse : HR_RANGE.human;
      if (nb >= rng[0] && nb <= rng[1]) {
        bpm = nb;
        const el = document.getElementById('bpmVal');
        el.textContent = bpm;
        el.classList.remove('pulse'); void el.offsetWidth; el.classList.add('pulse');
      }
    }
  }
}
function setLead(state, text) {
  const dot = document.getElementById('leadDot');
  dot.className = '';
  if (state === 'ok')   dot.classList.add('ok');
  if (state === 'conn') dot.classList.add('conn');
  document.getElementById('leadText').textContent = text;
}

// ====================================================
//  ПРОФИЛИ ЖИВОТНЫХ/ПАЦИЕНТОВ
// ====================================================
function renderProfileSelect() {
  const sel = document.getElementById('profileSelect');
  const cur = sel.value;
  sel.innerHTML = profiles.map(p => `<option value="${p.id}">${esc(p.name)}</option>`).join('')
    + `<option value="__new__">+ добавить...</option>`;
  if (profiles.some(p => p.id === cur)) sel.value = cur;
}
function onProfileChange() {
  const sel = document.getElementById('profileSelect');
  if (sel.value === '__new__') {
    const name = prompt('Имя животного/пациента:');
    sel.value = profiles[0] ? profiles[0].id : '';
    if (name && name.trim()) wsSend({cmd:'ADD_PROFILE', name: name.trim(), species: horseMode ? 'horse' : 'human'});
  }
}

// ====================================================
//  ЗАПИСЬ / МЕТКИ (управление с телефона, пишет ESP32 на SD)
// ====================================================
function toggleRec() {
  if (!recording) {
    const sel = document.getElementById('profileSelect');
    const pid = sel.value;
    if (!pid || pid === '__new__') { alert('Сначала выберите или добавьте профиль'); return; }
    wsSend({cmd:'START_REC', profile_id: pid, mode: horseMode ? 'horse' : 'human'});
  } else {
    wsSend({cmd:'STOP_REC'});
  }
}
function onRecStateChanged() {
  const btn = document.getElementById('recBtn');
  const mark = document.getElementById('markBtn');
  const flag = document.getElementById('recFlag');
  btn.textContent = recording ? '■ СТОП' : '⬤ ЗАПИСЬ';
  btn.classList.toggle('rec', recording);
  mark.classList.toggle('show', recording);
  flag.classList.toggle('on', recording);
}
function sendMark() {
  const label = prompt('Метка (необязательно):', '') || '';
  wsSend({cmd:'MARK', label: label.slice(0,31)});
}

// ====================================================
//  БИБЛИОТЕКА
// ====================================================
async function renderLibrary() {
  const el = document.getElementById('libList');
  const cnt = document.getElementById('libCount');
  cnt.textContent = sessions.length ? ` (${sessions.length})` : '';
  if (!sessions.length) { el.innerHTML = '<div class="emptyMsg">на SD-карте записей нет<br>нажмите ⬤ ЗАПИСЬ на вкладке МОНИТОР</div>'; return; }

  const cached = await dbGetAll();
  const cachedIds = new Set(cached.map(r => r.id));

  const byProfile = {};
  sessions.forEach(s => { (byProfile[s.profile_id] = byProfile[s.profile_id] || []).push(s); });

  el.innerHTML = Object.keys(byProfile).map(pid => {
    const prof = profiles.find(p => p.id === pid);
    const rows = byProfile[pid].slice().sort((a,b) => b.start_ms - a.start_ms).map(s => {
      const isCached = cachedIds.has(s.id);
      const si = selectedIds.indexOf(s.id);
      return `
        <div class="fileRow ${si!==-1?'sel'+si:''}" onclick="${isCached ? `toggleSel('${s.id}')` : `alert('Сначала выгрузите запись на телефон (↓)')`}">
          <div style="flex:1;overflow:hidden">
            <div class="fName">${fmtDate(s.start_ms)} · ${s.mode==='horse'?'🐴':'👤'}${s.marks && s.marks.length ? ' · ⚑'+s.marks.length : ''}</div>
            <div class="fMeta">${fmtDur(s.duration_s)} · ${s.sample_count} сэмплов</div>
          </div>
          <div class="fActs" onclick="event.stopPropagation()">
            <button class="iBtn ${isCached?'cached':''}" title="Выгрузить на телефон" onclick="exportSession('${s.id}')">${isCached?'✓':'↓'}</button>
          </div>
        </div>`;
    }).join('');
    return `<div class="profGroup"><div class="profHead"><span class="pname">${esc(prof ? prof.name : pid)}</span><span class="pcount">${byProfile[pid].length}</span></div>${rows}</div>`;
  }).join('');
}

async function exportSession(id) {
  try {
    const resp = await fetch(`/rec/${id}.csv`);
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const text = await resp.text();
    const meta = sessions.find(s => s.id === id) || {};
    const samples = [], leadflags = [];
    text.split('\n').forEach(line => {
      if (!line || line[0] === '#' || line.startsWith('time_s')) return;
      const parts = line.split(',');
      if (parts.length < 2) return;
      samples.push(parseInt(parts[1], 10));
      leadflags.push(parts[2] === '1');
    });
    await dbPut({ id, profile_id: meta.profile_id, mode: meta.mode, fs: FS, samples, leadflags, marks: meta.marks || [], duration: meta.duration_s, start_ms: meta.start_ms });
    renderLibrary();
  } catch (e) { alert('Не удалось выгрузить запись: ' + e); }
}

function toggleSel(id) {
  const idx = selectedIds.indexOf(id);
  if (idx !== -1) selectedIds.splice(idx, 1);
  else { if (selectedIds.length >= 3) selectedIds.shift(); selectedIds.push(id); }
  renderLibrary();
  updateCompView();
}

// ====================================================
//  АНАЛИЗ / СРАВНЕНИЕ (учитывает физиологическую АВ-блокаду лошади)
// ====================================================
async function updateCompView() {
  const ph = document.getElementById('compPlaceholder');
  if (!selectedIds.length) { ph.style.display = 'flex'; document.getElementById('statsPane').innerHTML = ''; clearCompCanvas(); return; }
  ph.style.display = 'none';
  const recs = [];
  for (const id of selectedIds) { const r = await dbGet(id); if (r) recs.push(r); }
  drawCompCanvas(recs);
  renderStats(recs);
}
function clearCompCanvas() {
  const wrap = document.getElementById('compWrap'); const cc = document.getElementById('compCanvas');
  cc.width = wrap.clientWidth; cc.height = wrap.clientHeight;
  const c = cc.getContext('2d'); c.fillStyle = '#000'; c.fillRect(0,0,cc.width,cc.height);
}
function drawCompCanvas(recs) {
  const wrap = document.getElementById('compWrap'); const cc = document.getElementById('compCanvas');
  const CW = wrap.clientWidth, CH = wrap.clientHeight; cc.width = CW; cc.height = CH;
  const c = cc.getContext('2d'); c.fillStyle = '#000'; c.fillRect(0,0,CW,CH); drawGrid(c,CW,CH);
  if (!recs.length) return;
  const showSamples = Math.min(FS*10, ...recs.map(r => r.samples.length));
  const step = Math.max(1, Math.floor(showSamples / CW));
  const n = recs.length;
  recs.forEach((rec, ti) => {
    const col = TRACK_COLORS[ti];
    const slice = rec.samples.slice(0, showSamples);
    let bl = slice.reduce((a,b)=>a+b,0)/slice.length;
    const vals = slice.map(s => s - bl);
    const maxV = Math.max(...vals.map(Math.abs)) || 1;
    const trackH = CH/n, trackMid = (ti+0.5)*trackH, amp = trackH*0.38;
    if (ti > 0) { c.strokeStyle = '#1a2a1a'; c.lineWidth = 1; c.setLineDash([3,6]); c.beginPath(); c.moveTo(0,ti*trackH); c.lineTo(CW,ti*trackH); c.stroke(); c.setLineDash([]); }
    c.beginPath(); c.strokeStyle = col; c.lineWidth = n===1?1.8:1.3; c.shadowColor = col; c.shadowBlur = n===1?5:2;
    let first = true;
    for (let i=0; i<slice.length; i+=step) { const x=(i/showSamples)*CW; const y=trackMid-(vals[i]/maxV)*amp; first?c.moveTo(x,y):c.lineTo(x,y); first=false; }
    c.stroke(); c.shadowBlur = 0;
    const prof = profiles.find(p => p.id === rec.profile_id);
    c.fillStyle = col; c.font = '9px monospace'; c.fillText((prof?prof.name:rec.profile_id).slice(0,22), 6, trackMid-amp-4);
  });
}
// Детектор пиков, устойчивый к пропущенным QRS (физиологическая АВ-блокада у лошади):
// не обнуляет ЧСС при нерегулярных RR, а выделяет доминирующий кластер интервалов.
function analyzeSignal(samples, mode) {
  if (!samples || samples.length < 20) return { bpm:0, hrv:0, peaks:0, dropRatio:0 };
  let bl = 2048;
  const vs = samples.map(s => { bl += 0.002*(s-bl); return (s-bl)/2048; });
  let pm=0, ps=0, lp=-999;
  const refSamp = (mode==='horse'?700:350)/1000*FS;
  const pkIdx = [];
  vs.forEach((v,i) => { ps=ps*0.97+v*0.03; pm=Math.max(pm*0.9995, Math.abs(v)); if (v > Math.max(0.15, pm*0.6) && (i-lp) > refSamp) { lp=i; pkIdx.push(i); } });
  if (pkIdx.length < 3) return { bpm:0, hrv:0, peaks:pkIdx.length, dropRatio:0 };

  const ivs = pkIdx.slice(1).map((p,i) => (p - pkIdx[i]) / FS * 1000);
  // Медиана как устойчивая к выбросам оценка типичного RR (в т.ч. при выпавших QRS)
  const sorted = ivs.slice().sort((a,b)=>a-b);
  const median = sorted[Math.floor(sorted.length/2)];
  // "Ядро" — интервалы близкие к медиане (обычный ритм), "длинные" — вероятные пропуски (АВ-блокада)
  const core = ivs.filter(iv => iv < median * 1.5);
  const longOnes = ivs.filter(iv => iv >= median * 1.5);
  const avgCore = core.length ? core.reduce((a,b)=>a+b,0)/core.length : median;
  const bpmVal = Math.round(60000 / avgCore);
  const hrv = core.length > 1 ? Math.round(Math.sqrt(core.reduce((a,b)=>a+(b-avgCore)**2,0)/core.length)) : 0;
  const rng = mode==='horse' ? HR_RANGE.horse : HR_RANGE.human;
  const bpmOk = bpmVal >= rng[0] && bpmVal <= rng[1];
  return { bpm: bpmOk ? bpmVal : 0, hrv, peaks: pkIdx.length, dropRatio: longOnes.length / ivs.length };
}
function renderStats(recs) {
  const pane = document.getElementById('statsPane');
  pane.innerHTML = recs.map((rec, ti) => {
    const col = TRACK_COLORS[ti];
    const stats = analyzeSignal(rec.samples, rec.mode);
    const prof = profiles.find(p => p.id === rec.profile_id);
    const avBlockNote = (rec.mode === 'horse' && stats.dropRatio > 0.15)
      ? `<div class="stWarn">⚠ регулярные пропуски QRS (${Math.round(stats.dropRatio*100)}%) — похоже на физиологическую АВ-блокаду в покое, не обязательно патология</div>` : '';
    return `
      <div class="statCard">
        <div class="stTitle" style="color:${col}">${esc((prof?prof.name:rec.profile_id||'').slice(0,14))}</div>
        <div class="stVal" style="color:${col}">${stats.bpm || '—'}</div>
        <div class="stSub">уд/мин · ${rec.mode==='horse'?'🐴':'👤'}</div>
        <div class="stSub">HRV ${stats.hrv?stats.hrv+'мс':'—'} · ${stats.peaks} пиков</div>
        <div class="stSub">${fmtDur(rec.duration)}${rec.marks && rec.marks.length ? ' · ⚑'+rec.marks.length : ''}</div>
        ${avBlockNote}
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
  document.getElementById('bpmVal').textContent = '---';
  peaks=[]; bpm=0; peakMax=0;
}

// ====================================================
//  МОДАЛЬНОЕ ОКНО (используется точечно, сейчас в основном alert/prompt)
// ====================================================
function hideModal() { document.getElementById('modal').style.display = 'none'; pendingCb = null; }
function modalOk() { if (pendingCb) pendingCb(document.getElementById('modalInput').value); hideModal(); }

// ====================================================
//  УТИЛИТЫ
// ====================================================
function fmtDur(sec) { sec = sec||0; const m = Math.floor(sec/60), s = Math.floor(sec%60); return m>0 ? `${m}м ${s}с` : `${s}с`; }
function fmtDate(ms) { return new Date(ms).toLocaleString('ru-RU', {day:'2-digit', month:'2-digit', hour:'2-digit', minute:'2-digit'}); }
function esc(s) { return String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c])); }
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
    case WStype_TEXT:
      handleWsText(num, payload, len);
      break;
    default:
      break;
  }
}

void wsReply(uint8_t num, JsonDocument& doc) {
  String out;
  serializeJson(doc, out);
  wsServer.sendTXT(num, out);
}
void wsError(uint8_t num, const char* msg) {
  JsonDocument doc; doc["type"] = "error"; doc["msg"] = msg;
  wsReply(num, doc);
}

// ============================================================
//  ИНДЕКС СЕССИЙ / ПРОФИЛИ
// ============================================================

void appendSessionToIndex() {
  JsonDocument doc;
  sdReadJson(INDEX_PATH, doc);
  JsonArray arr = doc.is<JsonArray>() ? doc.as<JsonArray>() : doc.to<JsonArray>();

  JsonObject s = arr.add<JsonObject>();
  s["id"]         = curSessionId;
  s["profile_id"] = curProfileId;
  s["mode"]       = curMode;
  s["start_ms"]   = sessionStartEpochMs;
  s["duration_s"] = sessionSampleCount / (double)SAMPLE_RATE;
  s["sample_count"] = sessionSampleCount;
  JsonArray ma = s["marks"].to<JsonArray>();
  for (uint8_t i = 0; i < marksCount; i++) {
    JsonObject m = ma.add<JsonObject>();
    m["i"] = marks[i].sampleIdx;
    m["label"] = marks[i].label;
  }

  sdWriteJson(INDEX_PATH, doc);
}

// ============================================================
//  ОБРАБОТКА КОМАНД (WS, текстовые JSON-фреймы)
// ============================================================

void handleWsText(uint8_t num, uint8_t* payload, size_t len) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, (const char*)payload, len);
  if (err) { wsError(num, "bad json"); return; }
  const char* cmd = doc["cmd"] | "";

  if (strcmp(cmd, "SET_TIME") == 0) {
    double t = doc["t"] | 0.0;
    phoneEpochOffsetMs = t - (double)millis();
    timeSynced = true;
  }
  else if (strcmp(cmd, "START_REC") == 0) {
    if (recording) { wsError(num, "already recording"); return; }
    curProfileId = String((const char*)(doc["profile_id"] | ""));
    curMode      = String((const char*)(doc["mode"] | "human"));
    curSessionId = String((uint32_t)(nowEpochMs() / 1000.0));
    sessionStartEpochMs = nowEpochMs();
    sessionSampleCount = 0;
    marksCount = 0;

    xSemaphoreTake(sdMux, portMAX_DELAY);
    if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
    String path = String(REC_DIR) + "/" + curSessionId + ".dat";
    sdFile = SD.open(path, FILE_WRITE);
    bool ok = (bool)sdFile;
    xSemaphoreGive(sdMux);

    if (!ok) { wsError(num, "sd open failed"); return; }

    xSemaphoreTake(ringMux, portMAX_DELAY);
    ringHead = 0; ringTail = 0;
    xSemaphoreGive(ringMux);

    recording = true;
    JsonDocument r; r["type"] = "rec_started"; r["id"] = curSessionId;
    wsReply(num, r);
  }
  else if (strcmp(cmd, "STOP_REC") == 0) {
    if (!recording) { wsError(num, "not recording"); return; }
    recording = false;
    delay(50); // дать sdTask() дописать хвост кольцевого буфера
    xSemaphoreTake(sdMux, portMAX_DELAY);
    if (sdFile) sdFile.close();
    xSemaphoreGive(sdMux);
    appendSessionToIndex();
    JsonDocument r; r["type"] = "rec_stopped"; r["id"] = curSessionId;
    wsReply(num, r);
  }
  else if (strcmp(cmd, "MARK") == 0) {
    if (!recording) { wsError(num, "not recording"); return; }
    if (marksCount < MAX_MARKS) {
      marks[marksCount].sampleIdx = sessionSampleCount;
      const char* label = doc["label"] | "";
      strncpy(marks[marksCount].label, label, MARK_LABEL_LEN - 1);
      marks[marksCount].label[MARK_LABEL_LEN - 1] = 0;
      marksCount++;
    }
  }
  else if (strcmp(cmd, "LIST") == 0) {
    sdEnsureFile(INDEX_PATH, "[]");
    xSemaphoreTake(sdMux, portMAX_DELAY);
    File f = SD.open(INDEX_PATH, FILE_READ);
    String content = "[]";
    if (f) { content = f.readString(); f.close(); }
    xSemaphoreGive(sdMux);
    String out = "{\"type\":\"list\",\"sessions\":" + content + "}";
    wsServer.sendTXT(num, out);
  }
  else if (strcmp(cmd, "LIST_PROFILES") == 0) {
    sdEnsureFile(PROFILES_PATH, "[]");
    xSemaphoreTake(sdMux, portMAX_DELAY);
    File f = SD.open(PROFILES_PATH, FILE_READ);
    String content = "[]";
    if (f) { content = f.readString(); f.close(); }
    xSemaphoreGive(sdMux);
    String out = "{\"type\":\"profiles\",\"profiles\":" + content + "}";
    wsServer.sendTXT(num, out);
  }
  else if (strcmp(cmd, "ADD_PROFILE") == 0) {
    JsonDocument pdoc;
    sdReadJson(PROFILES_PATH, pdoc);
    JsonArray arr = pdoc.is<JsonArray>() ? pdoc.as<JsonArray>() : pdoc.to<JsonArray>();
    JsonObject p = arr.add<JsonObject>();
    String id = String((uint32_t)(nowEpochMs()));
    p["id"] = id;
    p["name"] = (const char*)(doc["name"] | "?");
    p["species"] = (const char*)(doc["species"] | "horse");
    sdWriteJson(PROFILES_PATH, pdoc);

    xSemaphoreTake(sdMux, portMAX_DELAY);
    File f = SD.open(PROFILES_PATH, FILE_READ);
    String content = "[]"; if (f) { content = f.readString(); f.close(); }
    xSemaphoreGive(sdMux);
    String out = "{\"type\":\"profiles\",\"profiles\":" + content + "}";
    wsServer.broadcastTXT(out);
  }
  else if (strcmp(cmd, "RENAME_PROFILE") == 0) {
    JsonDocument pdoc;
    sdReadJson(PROFILES_PATH, pdoc);
    JsonArray arr = pdoc.is<JsonArray>() ? pdoc.as<JsonArray>() : pdoc.to<JsonArray>();
    String id = String((const char*)(doc["id"] | ""));
    for (JsonObject p : arr) { if (String((const char*)(p["id"] | "")) == id) p["name"] = (const char*)(doc["name"] | ""); }
    sdWriteJson(PROFILES_PATH, pdoc);
  }
}

// ============================================================
//  HTTP: СКАЧИВАНИЕ CSV (транскодирование бинарника на лету)
// ============================================================

void handleDownload(String id) {
  // Метаданные сессии (для заголовка CSV) — ищем в индексе
  JsonDocument idx;
  sdReadJson(INDEX_PATH, idx);
  String mode = "human", profileId = "";
  if (idx.is<JsonArray>()) {
    for (JsonObject s : idx.as<JsonArray>()) {
      if (String((const char*)(s["id"] | "")) == id) {
        mode = String((const char*)(s["mode"] | "human"));
        profileId = String((const char*)(s["profile_id"] | ""));
        break;
      }
    }
  }

  xSemaphoreTake(sdMux, portMAX_DELAY);
  File f = SD.open(String(REC_DIR) + "/" + id + ".dat", FILE_READ);
  bool ok = (bool)f;
  xSemaphoreGive(sdMux);

  if (!ok) { httpServer.send(404, "text/plain", "not found"); return; }

  httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  httpServer.send(200, "text/csv", "");
  String header = "# id=" + id + " profile_id=" + profileId + " mode=" + mode + " fs=" + String(SAMPLE_RATE) + " device=ESP32+AD8232+SD\ntime_s,raw_adc,lead_off\n";
  httpServer.sendContent(header);

  const int CHUNK = 250;
  uint16_t buf[CHUNK];
  uint32_t sampleIdx = 0;
  String out;
  out.reserve(4096);
  while (true) {
    xSemaphoreTake(sdMux, portMAX_DELAY);
    int n = f.available() ? f.read((uint8_t*)buf, CHUNK * 2) / 2 : 0;
    xSemaphoreGive(sdMux);
    if (n <= 0) break;
    out = "";
    for (int i = 0; i < n; i++) {
      uint16_t raw = buf[i];
      bool lo = (raw & 0x8000) != 0;
      uint16_t val = raw & 0x0FFF;
      out += String(sampleIdx / (float)SAMPLE_RATE, 4);
      out += ',';
      out += val;
      out += ',';
      out += (lo ? '1' : '0');
      out += '\n';
      sampleIdx++;
    }
    httpServer.sendContent(out);
  }
  xSemaphoreTake(sdMux, portMAX_DELAY);
  f.close();
  xSemaphoreGive(sdMux);
  httpServer.sendContent("");
}

// ============================================================
//  CAPTIVE PORTAL — чтобы Android не душил WebSocket на закрытой
//  AP-сети без интернета (см. историю в шапке файла)
// ============================================================

void handleCaptive() {
  httpServer.send(200, "text/html",
    "<html><head><meta http-equiv='refresh' content='0;url=http://192.168.4.1/'></head>"
    "<body>ECG Monitor</body></html>");
}

void handleNotFound() {
  String uri = httpServer.uri();
  if (uri.startsWith("/rec/") && uri.endsWith(".csv")) {
    String id = uri.substring(5, uri.length() - 4);
    handleDownload(id);
    return;
  }
  handleCaptive();
}

// ============================================================
//  SD-ЗАДАЧА (ядро 0) — сброс кольцевого буфера, независимая от
//  live-стрима и HTTP на ядре 1
// ============================================================

void sdTask(void* param) {
  const int CHUNK = 500;
  static uint16_t chunkBuf[CHUNK];
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(SD_FLUSH_MS));
    if (!recording) continue;

    uint32_t avail;
    xSemaphoreTake(ringMux, portMAX_DELAY);
    avail = ringHead - ringTail;
    if (avail > CHUNK) avail = CHUNK;
    for (uint32_t i = 0; i < avail; i++) chunkBuf[i] = ring[(ringTail + i) % RING_CAP];
    ringTail += avail;
    xSemaphoreGive(ringMux);

    if (avail > 0) {
      xSemaphoreTake(sdMux, portMAX_DELAY);
      if (sdFile) { sdFile.write((uint8_t*)chunkBuf, avail * 2); sdFile.flush(); }
      xSemaphoreGive(sdMux);
    }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ECG Monitor v3 Boot ===");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(LO_PLUS,  INPUT);
  pinMode(LO_MINUS, INPUT);

  ringMux = xSemaphoreCreateMutex();
  sdMux   = xSemaphoreCreateMutex();

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("[SD] ОШИБКА инициализации SD-карты! Запись работать не будет.");
  } else {
    Serial.println("[SD] Инициализирована");
    sdEnsureFile(INDEX_PATH, "[]");
    sdEnsureFile(PROFILES_PATH, "[]");
    if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  apIP = WiFi.softAPIP();
  Serial.printf("[WiFi] AP: %s  IP: %s\n", AP_SSID, apIP.toString().c_str());

  dnsServer.start(53, "*", apIP);

  httpServer.on("/", HTTP_GET, []() {
    httpServer.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  // Пробники captive-portal разных ОС — отвечаем "portal" вместо честного
  // 204/200, чтобы система не считала сеть "без интернета" и не душила WS.
  httpServer.on("/generate_204", HTTP_GET, handleCaptive);
  httpServer.on("/gen_204", HTTP_GET, handleCaptive);
  httpServer.on("/ncsi.txt", HTTP_GET, handleCaptive);
  httpServer.on("/connecttest.txt", HTTP_GET, handleCaptive);
  httpServer.on("/hotspot-detect.html", HTTP_GET, handleCaptive);
  httpServer.on("/library/test/success.html", HTTP_GET, handleCaptive);
  httpServer.onNotFound(handleNotFound);
  httpServer.begin();
  Serial.println("[HTTP] Запущен на порту 80");

  wsServer.begin();
  wsServer.onEvent(onWsEvent);
  Serial.println("[WS]   Запущен на порту 81");

  xTaskCreatePinnedToCore(sdTask, "sdTask", 8192, NULL, 1, NULL, 0);
  Serial.println("[SD]   Задача записи запущена на ядре 0");

  sampTimer = timerBegin(1000000);
  timerAttachInterrupt(sampTimer, &onTimer);
  timerAlarm(sampTimer, 1000000UL / SAMPLE_RATE, true, 0);
  Serial.printf("[ADC]  Таймер %d Гц запущен\n", SAMPLE_RATE);

  Serial.println("=== Готово! http://192.168.4.1 ===\n");
}

// ============================================================
//  LOOP (ядро 1) — сэмплирование, live-стрим, HTTP/WS, DNS
// ============================================================
static uint8_t packet[BATCH_SIZE * 2 + 1];

void loop() {
  httpServer.handleClient();
  wsServer.loop();
  dnsServer.processNextRequest();

  if (sampleTick) {
    sampleTick = false;
    uint16_t raw = (uint16_t)analogRead(ECG_PIN);
    bool leadOff = digitalRead(LO_PLUS) | digitalRead(LO_MINUS);

    portENTER_CRITICAL(&timerMux);
    adcBuf[adcIdx++] = raw;
    if (adcIdx >= BATCH_SIZE) { adcIdx = 0; pktReady = true; }
    portEXIT_CRITICAL(&timerMux);

    if (recording) {
      uint16_t packed = (raw & 0x0FFF) | (leadOff ? 0x8000 : 0);
      // Не ждём мьютекс здесь — время-критичный путь. Если занят (крайне
      // редко, sdTask как раз дренирует буфер), просто пропускаем один
      // сэмпл в кольцевой буфер, а не блокируем сэмплирование.
      if (xSemaphoreTake(ringMux, 0) == pdTRUE) {
        if (ringHead - ringTail < RING_CAP) { ring[ringHead % RING_CAP] = packed; ringHead++; }
        xSemaphoreGive(ringMux);
      }
      sessionSampleCount++;
    }
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
