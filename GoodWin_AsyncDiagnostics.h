#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_heap_caps.h>

// Bounded asynchronous diagnostics. This is deliberately NOT a shell:
// only DNS lookup, TCP connect and a bounded HTTP/1.1 GET are accepted.
namespace GoodWinDiagnostics {
static constexpr uint8_t JOB_CAPACITY = 4;
static constexpr size_t HOST_CAPACITY = 96;
static constexpr size_t OUTPUT_CAPACITY = 320;

enum JobState : uint8_t { EMPTY = 0, QUEUED = 1, RUNNING = 2, COMPLETED = 3, FAILED = 4 };
enum JobOperation : uint8_t { OP_DNS = 1, OP_TCP = 2, OP_HTTP = 3 };

struct Job {
  uint32_t id;
  uint32_t startedAt;
  uint32_t elapsedMs;
  uint16_t port;
  uint8_t state;
  uint8_t operation;
  char host[HOST_CAPACITY];
  char output[OUTPUT_CAPACITY];
};

static Job jobs[JOB_CAPACITY];
static QueueHandle_t queueHandle = nullptr;
static TaskHandle_t taskHandle = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t nextJobId = 1;
static uint8_t nextSlot = 0;

static bool validHost(const String &host) {
  if (host.isEmpty() || host.length() >= HOST_CAPACITY) return false;
  if (host[0] == '.' || host[host.length() - 1] == '.') return false;
  for (size_t i = 0; i < host.length(); ++i) {
    const char c = host[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '.' || c == '-')) return false;
  }
  return true;
}

static String jsonEscape(const char *value) {
  String out;
  if (!value) return out;
  for (const char *p = value; *p; ++p) {
    if (*p == '"' || *p == '\\') { out += '\\'; out += *p; }
    else if ((uint8_t)*p >= 0x20) out += *p;
  }
  return out;
}

static void setJobResult(uint8_t slot, JobState state, const char *output, uint32_t elapsed) {
  portENTER_CRITICAL(&mux);
  jobs[slot].state = state;
  jobs[slot].elapsedMs = elapsed;
  strlcpy(jobs[slot].output, output ? output : "", sizeof(jobs[slot].output));
  portEXIT_CRITICAL(&mux);
}

static void diagnosticsWorker(void *) {
  uint8_t slot = 0;
  for (;;) {
    if (xQueueReceive(queueHandle, &slot, portMAX_DELAY) != pdTRUE || slot >= JOB_CAPACITY) continue;

    Job job;
    portENTER_CRITICAL(&mux);
    job = jobs[slot];
    jobs[slot].state = RUNNING;
    jobs[slot].startedAt = millis();
    portEXIT_CRITICAL(&mux);

    const uint32_t started = millis();
    char output[OUTPUT_CAPACITY] = {0};
    IPAddress ip;
    const bool resolved = WiFi.status() == WL_CONNECTED && WiFi.hostByName(job.host, ip, 3000) == 1;

    if (WiFi.status() != WL_CONNECTED) {
      strlcpy(output, "ERROR: Wi-Fi is disconnected", sizeof(output));
      setJobResult(slot, FAILED, output, millis() - started);
      continue;
    }
    if (!resolved) {
      snprintf(output, sizeof(output), "DNS FAILED: %s (timeout or name not found)", job.host);
      setJobResult(slot, FAILED, output, millis() - started);
      continue;
    }

    if (job.operation == OP_DNS) {
      snprintf(output, sizeof(output), "DNS OK: %s -> %s", job.host, ip.toString().c_str());
      setJobResult(slot, COMPLETED, output, millis() - started);
      continue;
    }

    WiFiClient client;
    client.setTimeout(1200);
    const bool connected = client.connect(ip, job.port, 2500);
    if (!connected) {
      snprintf(output, sizeof(output), "TCP FAILED: %s:%u (%s)", job.host, job.port, ip.toString().c_str());
      setJobResult(slot, FAILED, output, millis() - started);
      continue;
    }

    if (job.operation == OP_TCP) {
      snprintf(output, sizeof(output), "TCP OK: %s:%u (%s)", job.host, job.port, ip.toString().c_str());
      client.stop();
      setJobResult(slot, COMPLETED, output, millis() - started);
      continue;
    }

    // HTTP mode is intentionally plain HTTP on port 80 only. HTTPS is not
    // silently downgraded and certificate validation is not bypassed.
    client.print("GET / HTTP/1.1\r\nHost: ");
    client.print(job.host);
    client.print("\r\nConnection: close\r\nUser-Agent: GoodWinSmartHub-Diagnostics\r\n\r\n");
    const uint32_t deadline = millis() + 1800UL;
    String statusLine;
    while (client.connected() && !client.available() && (int32_t)(deadline - millis()) > 0) {
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (client.available()) statusLine = client.readStringUntil('\n');
    client.stop();
    statusLine.trim();
    if (statusLine.startsWith("HTTP/")) {
      snprintf(output, sizeof(output), "HTTP RESPONSE: %s", statusLine.c_str());
      setJobResult(slot, COMPLETED, output, millis() - started);
    } else {
      strlcpy(output, "HTTP FAILED: no valid HTTP status line (for HTTPS use TCP port 443 test)", sizeof(output));
      setJobResult(slot, FAILED, output, millis() - started);
    }
  }
}

static void begin() {
  if (taskHandle) return;
  queueHandle = xQueueCreate(JOB_CAPACITY, sizeof(uint8_t));
  if (!queueHandle) return;
  if (xTaskCreatePinnedToCore(diagnosticsWorker, "hubDiag", 4096, nullptr, 1, &taskHandle, 0) != pdPASS) {
    taskHandle = nullptr;
    vQueueDelete(queueHandle);
    queueHandle = nullptr;
  }
}

static void handleCreate(WebServer &server) {
  if (!queueHandle) {
    server.send(503, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"diagnostics worker unavailable\"}");
    return;
  }
  const String opText = server.arg("op");
  const String host = server.arg("host");
  const String portText = server.arg("port");
  if (!validHost(host)) {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"invalid host; use hostname or IPv4 address\"}");
    return;
  }

  uint8_t operation = 0;
  if (opText == "dns") operation = OP_DNS;
  else if (opText == "tcp") operation = OP_TCP;
  else if (opText == "http") operation = OP_HTTP;
  else {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"allowed operations: dns, tcp, http; no shell, ping or traceroute yet\"}");
    return;
  }

  uint32_t portValue = operation == OP_HTTP ? 80U : (portText.length() ? (uint32_t)portText.toInt() : 443U);
  if (operation != OP_DNS && (portValue < 1 || portValue > 65535 || (operation == OP_HTTP && portValue != 80))) {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"invalid port; HTTP is limited to port 80\"}");
    return;
  }

  int selected = -1;
  portENTER_CRITICAL(&mux);
  for (uint8_t n = 0; n < JOB_CAPACITY; ++n) {
    const uint8_t candidate = (uint8_t)((nextSlot + n) % JOB_CAPACITY);
    if (jobs[candidate].state == EMPTY || jobs[candidate].state == COMPLETED || jobs[candidate].state == FAILED) {
      selected = candidate;
      nextSlot = (uint8_t)((candidate + 1) % JOB_CAPACITY);
      break;
    }
  }
  if (selected >= 0) {
    Job &job = jobs[selected];
    memset(&job, 0, sizeof(job));
    job.id = nextJobId++;
    if (nextJobId == 0) nextJobId = 1;
    job.state = QUEUED;
    job.operation = operation;
    job.port = (uint16_t)portValue;
    strlcpy(job.host, host.c_str(), sizeof(job.host));
    strlcpy(job.output, "Queued", sizeof(job.output));
  }
  portEXIT_CRITICAL(&mux);

  if (selected < 0) {
    server.send(429, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"all diagnostic slots are busy\"}");
    return;
  }

  const uint8_t slot = (uint8_t)selected;
  if (xQueueSend(queueHandle, &slot, 0) != pdTRUE) {
    portENTER_CRITICAL(&mux);
    jobs[slot].state = EMPTY;
    portEXIT_CRITICAL(&mux);
    server.send(429, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"diagnostics queue is full\"}");
    return;
  }

  uint32_t id = 0;
  portENTER_CRITICAL(&mux);
  id = jobs[slot].id;
  portEXIT_CRITICAL(&mux);
  String json = "{\"ok\":true,\"id\":" + String(id) + ",\"state\":\"queued\"}";
  server.send(202, "application/json; charset=utf-8", json);
}

static void handleGet(WebServer &server) {
  const uint32_t id = (uint32_t)server.arg("id").toInt();
  if (!id) {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"missing job id\"}");
    return;
  }
  Job snapshot;
  bool found = false;
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < JOB_CAPACITY; ++i) {
    if (jobs[i].id == id && jobs[i].state != EMPTY) {
      snapshot = jobs[i];
      found = true;
      break;
    }
  }
  portEXIT_CRITICAL(&mux);
  if (!found) {
    server.send(404, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"job not found or expired\"}");
    return;
  }
  static const char *states[] = {"empty", "queued", "running", "completed", "failed"};
  String json = "{\"ok\":true,\"id\":" + String(snapshot.id) +
    ",\"state\":\"" + states[snapshot.state] + "\",\"op\":" + String(snapshot.operation) +
    ",\"host\":\"" + jsonEscape(snapshot.host) + "\",\"port\":" + String(snapshot.port) +
    ",\"elapsedMs\":" + String(snapshot.elapsedMs) +
    ",\"output\":\"" + jsonEscape(snapshot.output) + "\"}";
  server.send(200, "application/json; charset=utf-8", json);
}

static void registerRoutes(WebServer &server) {
  server.on("/api/terminal/jobs", HTTP_POST, [&server]() { handleCreate(server); });
  server.on("/api/terminal/jobs", HTTP_GET, [&server]() { handleGet(server); });
}

static const char DASHBOARD_HTML[] PROGMEM = R"GWHTML(
<!doctype html><html lang="ru"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GoodWin Smart Hub · Control Plane</title>
<style>
:root{color-scheme:dark;--bg:#080d18;--panel:#111a2b;--panel2:#151f33;--line:#25324a;--muted:#8fa1bd;--text:#e8eef9;--cyan:#52d6e8;--green:#4cdb9a;--amber:#ffbd59;--red:#ff6b78}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(ellipse at 10% 0,#162845 0,transparent 45%),var(--bg);color:var(--text);font:14px/1.45 Inter,ui-sans-serif,system-ui,Segoe UI,Arial}
header{display:flex;align-items:center;justify-content:space-between;padding:22px clamp(16px,4vw,48px);border-bottom:1px solid var(--line);background:#09111fe8;position:sticky;top:0;z-index:2;backdrop-filter:blur(12px)}
.brand{font-size:18px;font-weight:750;letter-spacing:.02em}.brand span{color:var(--cyan)}.sub{color:var(--muted);font-size:12px;margin-top:3px}
.status{display:flex;gap:8px;align-items:center;color:var(--muted);font-size:12px}.dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 12px currentColor}
main{max-width:1440px;margin:auto;padding:28px clamp(14px,4vw,48px) 60px}.section{display:flex;align-items:end;justify-content:space-between;margin:26px 0 14px}.section h2{margin:0;font-size:16px}.section small{color:var(--muted)}
.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}.card{background:linear-gradient(145deg,var(--panel),#0e1726);border:1px solid var(--line);border-radius:16px;padding:18px;min-width:0;box-shadow:0 12px 32px #0002}
.card.wide{grid-column:span 2}.card.full{grid-column:1/-1}.kicker{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:var(--muted)}.value{font-size:30px;font-weight:750;letter-spacing:-.04em;margin:7px 0 2px}.unit{font-size:13px;color:var(--muted);font-weight:500}.hint{font-size:12px;color:var(--muted)}.gauge{height:7px;background:#26354c;border-radius:20px;margin-top:14px;overflow:hidden}.gauge i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--cyan),var(--green));border-radius:20px;transition:width .35s}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:14px}.metricline{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #ffffff0d}.metricline:last-child{border:0}.metricline span:first-child{color:var(--muted)}
.pill{display:inline-flex;align-items:center;gap:6px;border:1px solid var(--line);border-radius:30px;padding:4px 9px;font-size:11px;color:var(--muted)}.pill.ok{color:var(--green);border-color:#27654f}.pill.warn{color:var(--amber);border-color:#6a512a}
.toolbar{display:flex;gap:8px;flex-wrap:wrap}button,select,input{font:inherit;color:var(--text);background:#0b1423;border:1px solid var(--line);border-radius:9px;padding:10px 12px}button{cursor:pointer;background:#123348;border-color:#20516a}button:hover{filter:brightness(1.2)}input{min-width:0;flex:1}select{min-width:105px}
pre{margin:12px 0 0;background:#070c14;border:1px solid var(--line);border-radius:10px;padding:12px;min-height:86px;white-space:pre-wrap;overflow-wrap:anywhere;color:#b9d4e7;font:12px/1.5 ui-monospace,Consolas,monospace}
.bar{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.bar>*{flex:1}.bar button{flex:0 0 auto}.footer{color:var(--muted);font-size:11px;margin-top:24px}
@media(max-width:900px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.card.wide{grid-column:span 2}}
@media(max-width:560px){header{padding:15px}.grid{grid-template-columns:1fr}.card.wide{grid-column:span 1}.pair{grid-template-columns:1fr}.value{font-size:26px}}
</style></head><body>
<header><div><div class="brand">GoodWin <span>Smart Hub</span></div><div class="sub">Control plane · local device telemetry</div></div><div class="status"><i class="dot" id="liveDot"></i><span id="liveText">Подключение…</span></div></header>
<main>
<div class="section"><h2>Обзор системы</h2><small id="updated">Ожидание телеметрии</small></div>
<div class="grid">
<div class="card"><div class="kicker">Внутренняя температура</div><div class="value" id="inTemp">—<span class="unit"> °C</span></div><div class="gauge"><i id="tempGauge"></i></div><div class="hint">AM2302 · помещение</div></div>
<div class="card"><div class="kicker">Влажность внутри</div><div class="value" id="inHum">—<span class="unit"> %</span></div><div class="gauge"><i id="humGauge"></i></div><div class="hint">Относительная влажность</div></div>
<div class="card"><div class="kicker">Температура снаружи</div><div class="value" id="outTemp">—<span class="unit"> °C</span></div><div class="gauge"><i id="outGauge"></i></div><div class="hint" id="weatherSource">Погода · ожидание</div></div>
<div class="card"><div class="kicker">Свободная память</div><div class="value" id="heap">—<span class="unit"> KB</span></div><div class="gauge"><i id="heapGauge"></i></div><div class="hint" id="heapHint">Largest block: —</div></div>
<div class="card wide"><div class="kicker">Сеть и подключение</div><div class="pair"><div><div class="metricline"><span>Wi-Fi</span><b id="wifi">—</b></div><div class="metricline"><span>RSSI</span><b id="rssi">—</b></div><div class="metricline"><span>IP</span><b id="ip">—</b></div></div><div><div class="metricline"><span>Matter</span><b id="matter">—</b></div><div class="metricline"><span>Fabric</span><b id="fabric">—</b></div><div class="metricline"><span>MQTT</span><b id="mqtt">—</b></div></div></div></div>
<div class="card wide"><div class="kicker">Контроллер и дисплей</div><div class="pair"><div><div class="metricline"><span>CPU</span><b id="cpu">—</b></div><div class="metricline"><span>Чип</span><b id="chip">—</b></div><div class="metricline"><span>Uptime</span><b id="uptime">—</b></div></div><div><div class="metricline"><span>TFT FPS</span><b id="fps">—</b></div><div class="metricline"><span>Draw avg</span><b id="draw">—</b></div><div class="metricline"><span>Температура чипа</span><b id="chipTemp">—</b></div></div></div></div>
<div class="card wide"><div class="kicker">Локальный AI · adaptive agent</div><div class="value" id="aiMode" style="font-size:20px">ESP32 adaptive</div><div class="metricline"><span>События</span><b id="aiEvents">—</b></div><div class="metricline"><span>Аномалии климата</span><b id="aiAnomalies">—</b></div><div class="metricline"><span>Выбор источника погоды</span><b id="aiProvider">—</b></div><div class="hint" id="modelStatus">Micro-LM: проверка интеграции отдельно; генеративная модель пока не загружена.</div></div>
<div class="card wide"><div class="kicker">Погода · состояние провайдеров</div><div id="providers"></div><div class="toolbar" style="margin-top:12px"><button onclick="post('/weather/refresh')">Обновить погоду</button></div></div>
<div class="card full"><div class="section" style="margin:0 0 12px"><h2>Network diagnostics · web terminal</h2><span class="pill">ограниченные команды</span></div><div class="bar"><select id="op"><option value="dns">DNS lookup</option><option value="tcp">TCP connect</option><option value="http">HTTP GET</option></select><input id="host" value="wttr.in" placeholder="hostname / IPv4"><input id="port" type="number" value="443" min="1" max="65535" style="max-width:110px"><button onclick="runDiag()">Запустить</button></div><pre id="terminal">Доступно: dns, tcp (включая порт 443), http (только порт 80). Произвольная командная оболочка, ping и traceroute не включены.</pre></div>
</div>
<div class="footer">GoodWin Smart Hub · локальная панель устройства · обновление телеметрии каждые 3 секунды</div>
</main>
<script>
const $=id=>document.getElementById(id);const fmt=(v,d=1)=>v==null?'—':Number(v).toFixed(d);
async function get(url){const r=await fetch(url,{cache:'no-store'});if(!r.ok)throw Error(r.status);return r.json()}
async function post(url){try{await fetch(url,{method:'POST'})}catch(e){console.warn(e)}}
function gauge(id,v,max){$(id).style.width=Math.max(0,Math.min(100,(Number(v)||0)/max*100))+'%'}
async function refresh(){try{const [a,s,h]=await Promise.all([get('/api'),get('/system'),get('/hub/data')]);
$('liveText').textContent='ONLINE';$('liveDot').style.background='var(--green)';$('updated').textContent='Обновлено '+new Date().toLocaleTimeString();
$('wifi').textContent=a.wifi;$('rssi').textContent=a.rssi+' dBm';$('ip').textContent=a.ip;$('matter').textContent=a.matter;$('fabric').textContent=a.commissioned?'commissioned':'not commissioned';$('mqtt').textContent=a.mqttConnected?'connected':(a.mqttStatus||'offline');
$('inTemp').innerHTML=fmt(h.indoorTemp)+'<span class="unit"> °C</span>';$('inHum').innerHTML=fmt(h.indoorHumidity)+'<span class="unit"> %</span>';$('outTemp').innerHTML=fmt(h.outdoorTemp)+'<span class="unit"> °C</span>';
gauge('tempGauge',h.indoorTemp==null?0:Number(h.indoorTemp)+10,50);gauge('humGauge',h.indoorHumidity,100);gauge('outGauge',h.outdoorTemp==null?0:Number(h.outdoorTemp)+10,50);$('weatherSource').textContent=(h.weatherSource||'—')+' · '+(h.weatherStatus||'');
$('heap').innerHTML=(s.freeHeap/1024).toFixed(1)+'<span class="unit"> KB</span>';gauge('heapGauge',s.freeHeap,250000);$('heapHint').textContent='Largest block: '+Math.round(s.largestHeap/1024)+' KB · min: '+Math.round(s.minFreeHeap/1024)+' KB';
$('cpu').textContent=s.cpuMHz+' MHz';$('chip').textContent=s.chipModel+' rev '+s.chipRevision;$('uptime').textContent=Math.floor(s.uptime/3600)+'h '+Math.floor(s.uptime%3600/60)+'m';$('fps').textContent=s.tftFps;$('draw').textContent=s.tftDrawAvgUs+' µs';$('chipTemp').textContent=s.chipTempC==null?'—':fmt(s.chipTempC)+' °C';
const ai=h.ai||{};$('aiEvents').textContent=ai.events??'—';$('aiAnomalies').textContent=ai.anomalies??'—';$('aiProvider').textContent=h.weatherSource||'—';
$('providers').innerHTML=(ai.providers||[]).map((p,i)=>'<div class="metricline"><span>'+['Open-Meteo','wttr.in','MET Norway'][i]+'</span><b>'+(p.ok||0)+' / '+(p.fail||0)+' · '+(p.avgLatencyMs||0)+' ms</b></div>').join('');
}catch(e){$('liveText').textContent='OFFLINE';$('liveDot').style.background='var(--red)';}}
async function runDiag(){const op=$('op').value,host=$('host').value.trim(),port=$('port').value;const box=$('terminal');box.textContent='Отправка задания…';try{const body=new URLSearchParams({op,host,port});const r=await fetch('/api/terminal/jobs',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();if(!r.ok)throw Error(j.error||r.status);box.textContent='Задание #'+j.id+' · queued';for(let n=0;n<25;n++){await new Promise(x=>setTimeout(x,300));const q=await get('/api/terminal/jobs?id='+j.id);box.textContent='#'+q.id+' · '+q.state+'\n'+q.output+'\nВремя: '+q.elapsedMs+' ms';if(q.state==='completed'||q.state==='failed')break;}}catch(e){box.textContent='Ошибка: '+e.message;}}
refresh();setInterval(refresh,3000);
</script></body></html>
)GWHTML";

static void handleDashboard(WebServer &server) {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML);
}
} // namespace GoodWinDiagnostics
