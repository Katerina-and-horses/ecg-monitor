/*
 * ЭКГ Монитор — ESP32 + AD8232
 * WebSocket на порту 81 (стабильная библиотека arduinoWebSockets)
 *
 * БИБЛИОТЕКА (Arduino Library Manager):
 *   Поиск: "WebSockets"  →  автор: Markus Sattler  →  Install
 *   (AsyncTCP и ESPAsyncWebServer больше НЕ нужны)
 *
 * ПИНЫ:
 *   AD8232 OUTPUT → GPIO 34
 *   AD8232 LO+    → GPIO 26
 *   AD8232 LO-    → GPIO 25
 *
 * КАК ИСПОЛЬЗОВАТЬ:
 *   1. Загрузить прошивку
 *   2. WiFi → "ECG_Monitor"  пароль: ecg12345
 *   3. Chrome → http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ===== НАСТРОЙКИ =====
const char* AP_SSID     = "ECG_Monitor";
const char* AP_PASSWORD = "ecg12345";

const int ECG_PIN  = 34;
const int LO_PLUS  = 26;
const int LO_MINUS = 25;

const unsigned long SAMPLE_MS = 10;   // 100 Гц
// =====================

WebServer        httpServer(80);
WebSocketsServer wsServer(81);        // WS на порту 81

static const char INDEX_HTML[] PROGMEM = R"=====(<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ЭКГ Монитор</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
:root{--bg:#0d1117;--surf:#161b22;--surf2:#21262d;--brd:#30363d;--txt:#e6edf3;--muted:#8b949e;--ok:#3fb950;--err:#f85149;--warn:#d29922;--pri:#58a6ff}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--txt);font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;height:100dvh;display:flex;flex-direction:column;overflow:hidden}
header{background:var(--surf);border-bottom:1px solid var(--brd);padding:10px 16px;display:flex;align-items:center;justify-content:space-between;flex-shrink:0}
.logo{display:flex;align-items:center;gap:8px;font-weight:700;font-size:15px}
.srow{display:flex;align-items:center;gap:8px}
.dot{width:9px;height:9px;border-radius:50%;background:var(--err);transition:background .3s;flex-shrink:0}
.dot.ok{background:var(--ok);animation:pulse 2s infinite}
.dot.warn{background:var(--warn)}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
.stxt{font-size:12px;color:var(--muted)}
.stats{display:flex;background:var(--surf2);border-bottom:1px solid var(--brd);flex-shrink:0}
.stat{flex:1;padding:8px 10px;border-right:1px solid var(--brd);text-align:center}
.stat:last-child{border-right:none}
.sl{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:.6px;margin-bottom:2px}
.sv{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums;letter-spacing:-.5px}
.sv.rec{color:var(--err)}
.wrap{flex:1;padding:10px;min-height:0;position:relative}
#c{width:100%!important;height:100%!important;background:var(--surf);border-radius:8px;border:1px solid var(--brd)}
.badge{display:none;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);background:rgba(210,153,34,.15);border:1px solid var(--warn);color:var(--warn);padding:10px 20px;border-radius:8px;font-size:14px;font-weight:600;pointer-events:none;white-space:nowrap}
.badge.show{display:block}
footer{background:var(--surf);border-top:1px solid var(--brd);padding:10px 14px;display:flex;gap:10px;flex-shrink:0}
.btn{padding:13px 0;border:none;border-radius:8px;font-size:14px;font-weight:700;cursor:pointer;transition:all .2s;flex:1}
.btn-r{background:var(--ok);color:#000}
.btn-r.on{background:var(--err);color:#fff}
.btn-d{background:var(--pri);color:#000;flex:0 0 auto;padding:13px 18px}
.btn-d:disabled{opacity:.35;cursor:not-allowed}
</style>
</head>
<body>
<header>
  <div class="logo">
    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#00d9ff" stroke-width="2.5"><polyline points="22 12 18 12 15 21 9 3 6 12 2 12"/></svg>
    ЭКГ Монитор
  </div>
  <div class="srow">
    <div class="dot" id="dot"></div>
    <span class="stxt" id="stxt">Подключение…</span>
  </div>
</header>
<div class="stats">
  <div class="stat"><div class="sl">Пульс уд/мин</div><div class="sv" id="hr">—</div></div>
  <div class="stat"><div class="sl">Частота Гц</div><div class="sv" id="sps">—</div></div>
  <div class="stat"><div class="sl">Запись</div><div class="sv rec" id="dur">00:00</div></div>
</div>
<div class="wrap">
  <canvas id="c"></canvas>
  <div class="badge" id="badge">&#9888; Электрод не контактирует</div>
</div>
<footer>
  <button class="btn btn-r" id="rb" onclick="toggleRec()">&#9679; Запись</button>
  <button class="btn btn-d" id="db" onclick="dlCSV()" disabled>&#8595; CSV</button>
</footer>
<script>
const N=600,buf=new Array(N).fill(null);
const chart=new Chart(document.getElementById('c').getContext('2d'),{
  type:'line',
  data:{labels:new Array(N).fill(''),datasets:[{data:buf,borderColor:'#00d9ff',borderWidth:1.5,pointRadius:0,tension:0,fill:false,spanGaps:false}]},
  options:{animation:false,responsive:true,maintainAspectRatio:false,
    plugins:{legend:{display:false},tooltip:{enabled:false}},
    scales:{x:{display:false},y:{min:0,max:4095,ticks:{color:'#8b949e',font:{size:9},maxTicksLimit:6},grid:{color:'rgba(48,54,61,0.6)'},border:{color:'#30363d'}}}}
});
let hrBuf=[],lastPeakTs=0,prevVal=0;
function detectHR(val,ts){
  if(prevVal<2800&&val>=2800&&lastPeakTs>0){
    const bpm=Math.round(60000/(ts-lastPeakTs));
    if(bpm>25&&bpm<220){hrBuf.push(bpm);if(hrBuf.length>6)hrBuf.shift();
      document.getElementById('hr').textContent=Math.round(hrBuf.reduce((a,b)=>a+b)/hrBuf.length);}
  }
  if(prevVal<2800&&val>=2800)lastPeakTs=ts;
  prevVal=val;
}
let ws,rec=false,recData=[],recStart=0,recTimer=null,sps=0;
function connect(){
  ws=new WebSocket('ws://'+location.hostname+':81');
  ws.onopen=()=>{document.getElementById('dot').className='dot ok';document.getElementById('stxt').textContent='Подключено';};
  ws.onclose=()=>{document.getElementById('dot').className='dot';document.getElementById('stxt').textContent='Переподключение…';setTimeout(connect,2000);};
  ws.onerror=()=>ws.close();
  ws.onmessage=e=>{
    const p=e.data.split(',');if(p.length<3)return;
    const ts=parseInt(p[0]),val=parseInt(p[1]),lo=p[2]==='1';
    sps++;
    document.getElementById('badge').className='badge'+(lo?' show':'');
    if(lo){document.getElementById('dot').className='dot warn';document.getElementById('stxt').textContent='Электрод не контактирует';buf.shift();buf.push(null);}
    else{document.getElementById('dot').className='dot ok';document.getElementById('stxt').textContent='Подключено';buf.shift();buf.push(val);detectHR(val,ts);if(rec)recData.push(new Date().toISOString()+','+ts+','+val);}
    chart.update('none');
  };
}
setInterval(()=>{document.getElementById('sps').textContent=sps||'—';sps=0;},1000);
function toggleRec(){
  const rb=document.getElementById('rb'),db=document.getElementById('db');
  if(!rec){rec=true;recData=[];recStart=Date.now();rb.textContent='■ Стоп';rb.classList.add('on');db.disabled=true;
    recTimer=setInterval(()=>{const s=Math.floor((Date.now()-recStart)/1000);document.getElementById('dur').textContent=String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');},500);
  }else{rec=false;clearInterval(recTimer);rb.textContent='● Запись';rb.classList.remove('on');db.disabled=recData.length===0;}
}
function dlCSV(){
  const d=new Date(),z=n=>String(n).padStart(2,'0');
  const name='ECG_'+d.getFullYear()+z(d.getMonth()+1)+z(d.getDate())+'_'+z(d.getHours())+z(d.getMinutes())+z(d.getSeconds())+'.csv';
  const blob=new Blob(['datetime_iso,esp32_uptime_ms,adc_raw\n'+recData.join('\n')],{type:'text/csv'});
  const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;a.click();URL.revokeObjectURL(a.href);
}
connect();
</script>
</body>
</html>
)=====";

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = wsServer.remoteIP(num);
    Serial.printf("[WS] Клиент #%u подключился (%d.%d.%d.%d)\n", num, ip[0],ip[1],ip[2],ip[3]);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] Клиент #%u отключился\n", num);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ЭКГ Монитор v2 ===");

  pinMode(LO_PLUS,  INPUT);
  pinMode(LO_MINUS, INPUT);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP:  %s\n", AP_SSID);
  Serial.printf("IP:  %s\n", WiFi.softAPIP().toString().c_str());

  httpServer.on("/", []() {
    httpServer.send_P(200, "text/html", INDEX_HTML);
  });
  httpServer.begin();
  Serial.println("HTTP: порт 80  |  WS: порт 81");
  Serial.println("Откройте: http://192.168.4.1");

  wsServer.begin();
  wsServer.onEvent(wsEvent);
}

void loop() {
  httpServer.handleClient();
  wsServer.loop();

  static unsigned long lastSample = 0;
  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    bool lo = (digitalRead(LO_PLUS) == HIGH) || (digitalRead(LO_MINUS) == HIGH);
    int  val = lo ? 0 : analogRead(ECG_PIN);
    char msg[48];
    snprintf(msg, sizeof(msg), "%lu,%d,%d", now, val, lo ? 1 : 0);
    wsServer.broadcastTXT(msg);
  }
}
