/*
  ESP32 Mobile Robot - Motor Executor (MODE ACCESS POINT) + WEB CONTROL
  --------------------------------------------------------
  - WebSocket (port 81): terima {"left": -255..255, "right": -255..255}
  - Web server (port 80): halaman kontrol berisi joystick virtual
    (analog, bisa maju/mundur/belok sekaligus) dan slider kecepatan maks.
  Buka http://192.168.4.1 dari HP/laptop yang terhubung ke WiFi ROBOT_ESP32.

  Library: WebSockets (Markus Sattler), ArduinoJson (Benoit Blanchon)
  WebServer.h sudah bawaan ESP32 Arduino Core.
  ESP32 Arduino Core 3.x ke atas (analogWrite native).
*/

#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ---------- KONFIGURASI ACCESS POINT ----------
const char* AP_SSID     = "ROBOT_ESP32";
const char* AP_PASSWORD = "12345678";   // minimal 8 karakter
const int   AP_CHANNEL  = 6;
const int   AP_MAX_CONN = 2;

// ---------- PIN MOTOR DRIVER (MX1508) ----------
#define IN1 14   // Motor kiri
#define IN2 27   // Motor kiri
#define IN3 26   // Motor kanan
#define IN4 25   // Motor kanan

// ---------- TIMING ----------
const unsigned long COMMAND_TIMEOUT_MS = 800;
const unsigned long CLIENT_IDLE_MS     = 3000;
const unsigned long HEAP_LOG_MS        = 5000;

WebServer        server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

unsigned long lastCommandTime = 0;
unsigned long lastRx[WEBSOCKETS_SERVER_CLIENT_MAX];
unsigned long lastHousekeeping = 0;
unsigned long lastHeapLog = 0;

// ---------- HALAMAN WEB KONTROL ----------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Joystick Robot ESP32</title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html, body { margin: 0; height: 100%; background: #111827; color: #f9fafb;
               font-family: system-ui, sans-serif; user-select: none; -webkit-user-select: none;
               overflow: hidden; }
  body { display: flex; flex-direction: column; align-items: center;
         justify-content: center; gap: 22px; padding: 16px; }
  h1 { font-size: 20px; margin: 0; }
  #status { font-size: 13px; padding: 4px 12px; border-radius: 999px; background: #991b1b; }
  #status.on { background: #166534; }

  #base { position: relative; width: min(280px, 80vw); aspect-ratio: 1; border-radius: 50%;
          background: radial-gradient(circle, #1f2937 0%, #111827 70%);
          border: 3px solid #374151; touch-action: none; }
  #base::before, #base::after { content: ""; position: absolute; background: #374151; }
  #base::before { left: 50%; top: 8%; width: 2px; height: 84%; transform: translateX(-50%); }
  #base::after  { top: 50%; left: 8%; height: 2px; width: 84%; transform: translateY(-50%); }
  #knob { position: absolute; left: 50%; top: 50%; width: 34%; aspect-ratio: 1; border-radius: 50%;
          background: #2563eb; border: 3px solid #93c5fd; box-shadow: 0 4px 14px rgba(0,0,0,.5);
          transform: translate(-50%, -50%); pointer-events: none; z-index: 1; }
  #base.active #knob { background: #3b82f6; }

  #readout { font-size: 14px; color: #9ca3af; font-variant-numeric: tabular-nums; }
  .speed { width: min(320px, 90vw); text-align: center; }
  .speed input { width: 100%; height: 36px; }
  #spdVal { font-size: 22px; font-weight: 700; }
</style>
</head>
<body>
  <h1>Joystick Robot</h1>
  <div id="status">Terputus</div>

  <div id="base"><div id="knob"></div></div>
  <div id="readout">L: 0 &nbsp; R: 0</div>

  <div class="speed">
    <div>Kecepatan maks: <span id="spdVal">200</span> / 255</div>
    <input type="range" id="spd" min="60" max="255" value="200">
  </div>

<script>
  let ws;
  let jx = 0, jy = 0;          // posisi joystick, -1..1 (jy: atas = +1 = maju)
  let speed = 200;
  const DEADZONE = 0.08;

  const statusEl = document.getElementById('status');
  const base = document.getElementById('base');
  const knob = document.getElementById('knob');
  const readout = document.getElementById('readout');
  const spd = document.getElementById('spd');
  const spdVal = document.getElementById('spdVal');

  function connect() {
    ws = new WebSocket('ws://' + location.hostname + ':81/');
    ws.onopen  = () => { statusEl.textContent = 'Terhubung'; statusEl.classList.add('on'); };
    ws.onclose = () => { statusEl.textContent = 'Terputus'; statusEl.classList.remove('on'); setTimeout(connect, 1000); };
    ws.onerror = () => ws.close();
  }
  connect();

  // Arcade drive: gabungkan maju/mundur (jy) dan belok (jx) jadi kiri/kanan
  function mix() {
    let x = Math.abs(jx) < DEADZONE ? 0 : jx;
    let y = Math.abs(jy) < DEADZONE ? 0 : jy;
    let l = y - x, r = y + x;
    const m = Math.max(1, Math.abs(l), Math.abs(r));
    l /= m; r /= m;
    return [Math.round(l * speed), Math.round(r * speed)];
  }

  // Kirim tiap 50 ms (juga saat diam = keepalive)
  setInterval(() => {
    const [l, r] = mix();
    readout.innerHTML = 'L: ' + l + ' &nbsp; R: ' + r;
    if (ws && ws.readyState === 1) ws.send(JSON.stringify({ left: l, right: r }));
  }, 50);

  spd.addEventListener('input', () => { speed = +spd.value; spdVal.textContent = speed; });

  // ---- Joystick ----
  function setStick(x, y) {
    jx = x; jy = y;
    const R = base.clientWidth / 2;
    const maxTravel = R - knob.clientWidth / 2;
    knob.style.transform =
      'translate(calc(-50% + ' + (x * maxTravel) + 'px), calc(-50% + ' + (-y * maxTravel) + 'px))';
  }

  function moveFromEvent(e) {
    const rect = base.getBoundingClientRect();
    const R = rect.width / 2;
    const maxTravel = R - knob.clientWidth / 2;
    let dx = e.clientX - (rect.left + R);
    let dy = e.clientY - (rect.top + R);
    const dist = Math.hypot(dx, dy);
    if (dist > maxTravel) { dx = dx / dist * maxTravel; dy = dy / dist * maxTravel; }
    setStick(dx / maxTravel, -dy / maxTravel);
  }

  let activeId = null;
  base.addEventListener('pointerdown', e => {
    e.preventDefault();
    activeId = e.pointerId;
    base.setPointerCapture(activeId);
    base.classList.add('active');
    moveFromEvent(e);
  });
  base.addEventListener('pointermove', e => { if (e.pointerId === activeId) moveFromEvent(e); });
  function release(e) {
    if (e.pointerId !== activeId) return;
    activeId = null;
    base.classList.remove('active');
    setStick(0, 0);       // kembali ke tengah = berhenti
  }
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev => base.addEventListener(ev, release));
  base.addEventListener('contextmenu', e => e.preventDefault());

  // ---- Keyboard (opsional): panah / WASD ----
  const keys = {};
  function kbUpdate() {
    if (activeId !== null) return;
    const x = (keys.ArrowRight || keys.d ? 1 : 0) - (keys.ArrowLeft || keys.a ? 1 : 0);
    const y = (keys.ArrowUp || keys.w ? 1 : 0) - (keys.ArrowDown || keys.s ? 1 : 0);
    setStick(x, y);
  }
  window.addEventListener('keydown', e => { keys[e.key] = true;  kbUpdate(); });
  window.addEventListener('keyup',   e => { keys[e.key] = false; kbUpdate(); });
  window.addEventListener('blur', () => { for (const k in keys) keys[k] = false; if (activeId === null) setStick(0, 0); });
</script>
</body>
</html>
)rawliteral";

// ---------- MOTOR ----------
void setMotorKiri(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(IN1, speed); analogWrite(IN2, 0); }
  else            { analogWrite(IN1, 0);     analogWrite(IN2, -speed); }
}

void setMotorKanan(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(IN3, speed); analogWrite(IN4, 0); }
  else            { analogWrite(IN3, 0);     analogWrite(IN4, -speed); }
}

void driveMotors(int left, int right) {
  setMotorKiri(left);
  setMotorKanan(right);
}

void stopMotors() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
}

// ---------- WEBSOCKET ----------
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u terputus (sisa: %u)\n", num, webSocket.connectedClients());
      if (webSocket.connectedClients() == 0) stopMotors();
      break;

    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u terhubung\n", num);
      lastRx[num] = millis();
      for (uint8_t i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
        if (i != num && webSocket.clientIsConnected(i)) {
          Serial.printf("[WS] Putuskan client lama #%u\n", i);
          webSocket.disconnect(i);
        }
      }
      break;

    case WStype_TEXT: {
      lastRx[num] = millis();
      StaticJsonDocument<64> doc;
      if (deserializeJson(doc, payload, length)) return;
      int l = doc["left"]  | 0;
      int r = doc["right"] | 0;
      driveMotors(l, r);
      // Hanya perintah non-nol yang mengaktifkan failsafe timer
      lastCommandTime = (l != 0 || r != 0) ? millis() : 0;
      break;
    }

    default:
      break;
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONN);

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  if (ok) {
    Serial.printf("\nAP aktif! SSID: %s  Channel: %d\n", AP_SSID, AP_CHANNEL);
    Serial.print("Buka di browser: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Gagal memulai Access Point!");
  }

  // Web server: halaman kontrol
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

// ---------- LOOP ----------
void loop() {
  webSocket.loop();      // jangan taruh delay() di loop ini
  server.handleClient();

  unsigned long now = millis();

  // Failsafe motor
  if (lastCommandTime > 0 && now - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopMotors();
    lastCommandTime = 0;
  }

  // Buang client yang diam terlalu lama
  if (now - lastHousekeeping > 1000) {
    lastHousekeeping = now;
    for (uint8_t i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; i++) {
      if (webSocket.clientIsConnected(i) && now - lastRx[i] > CLIENT_IDLE_MS) {
        Serial.printf("[WS] Client #%u idle, diputus\n", i);
        webSocket.disconnect(i);
      }
    }
  }

  // Monitor memori
  if (now - lastHeapLog > HEAP_LOG_MS) {
    lastHeapLog = now;
    Serial.printf("[SYS] heap=%u clients=%u\n", ESP.getFreeHeap(), webSocket.connectedClients());
  }
}
