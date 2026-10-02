/*
  ========================================================================
  AESTRA — Autonomous Smart Agricultural Monitoring Rover
  ========================================================================
  Built for: ESP32 Dev Board (USB Type-C)

  Functionalities implemented (per project PDF):
    1. Autonomous Navigation & Obstacle Avoidance (HC-SR04 + L298N motors)
       - Drives forward continuously until an obstacle is detected, stops
         briefly, turns (alternating right/left so it doesn't get stuck
         bouncing between the same two walls), then resumes forward.
       - Can be paused/resumed any time from the dashboard Start/Stop button.
    2. Intelligent Environmental & Soil Sensing (Soil moisture + LDR light
       sensor, with a servo that lowers a probe arm into the soil)
       - Runs automatically on a timer, OR can be driven manually from a
         slider on the dashboard (automatic cycle pauses while manual).
    3. Wireless Telemetry & IoT Dashboard Monitoring (ESP32 Wi-Fi web page)
       - Live sensor cards, Start/Stop control, manual servo slider,
         and 3 history charts (line / bar / pie) — all built with plain
         HTML/CSS/JS served from flash (PROGMEM), no external libraries,
         so it works fully offline in Access Point mode.

  NOT implemented (per your instruction — excluded from the final build):
    - Buzzer, Push Buttons, Potentiometer, manual/auto toggle switch

  ------------------------------------------------------------------------
  LIBRARIES TO INSTALL (Arduino Library Manager, or PlatformIO lib_deps):
    - "ESP32Servo"                by Kevin Harrington / Jonathan Nordbrook
    - "LiquidCrystal I2C"         by Frank de Brabander
    - "DHT sensor library"        by Adafruit
    - "Adafruit Unified Sensor"   by Adafruit   (dependency of the above)
    (WiFi.h and WebServer.h are already built into the ESP32 core)

  ------------------------------------------------------------------------
  PIN MAP (from "ESP32 Robot — FINAL Pin & Wiring Map"):
    GPIO 4   -> DHT11 DATA
    GPIO 5   -> Servo signal (soil probe arm)
    GPIO 14  -> L298N IN4
    GPIO 18  -> HC-SR04 TRIG
    GPIO 19  -> HC-SR04 ECHO
    GPIO 21  -> LCD SDA (I2C)
    GPIO 22  -> LCD SCL (I2C)
    GPIO 25  -> L298N IN1
    GPIO 26  -> L298N IN2
    GPIO 27  -> L298N IN3
    GPIO 34  -> Soil moisture sensor AO (input-only pin)
    GPIO 35  -> LDR analog reading (input-only pin)
    ENA / ENB on the L298N are left with their jumper caps ON (always
    enabled) — so motor speed is fixed at full power, and IN1..IN4 only
    control direction. No PWM / speed control is wired.
  ========================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------------------------------------------------------------------
// WI-FI CONFIG (Functionality 3: Wireless Telemetry & IoT Dashboard)
// ---------------------------------------------------------------------
#define USE_ACCESS_POINT_MODE true

const char* AP_SSID     = "Aestra-Rover";
const char* AP_PASSWORD = "aestra123";   // must be at least 8 characters

const char* STA_SSID     = "Think Tank _ 04";
const char* STA_PASSWORD = "thinktank14312";

WebServer server(80);

// ---------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------
#define DHT_PIN        4
#define DHT_TYPE       DHT11

#define SERVO_PIN      5

#define L298N_IN1      25   // LEFT channel
#define L298N_IN2      26   // LEFT channel
#define L298N_IN3      27   // RIGHT channel
#define L298N_IN4      14   // RIGHT channel

#define TRIG_PIN       18
#define ECHO_PIN       19

#define SOIL_PIN       34   // analog, input-only
#define LDR_PIN        35   // analog, input-only

#define LCD_ADDRESS    0x27
#define LCD_COLS       16
#define LCD_ROWS       2

// ---------------------------------------------------------------------
// TUNABLE CONSTANTS — calibrate these to your own hardware
// ---------------------------------------------------------------------
const int OBSTACLE_DISTANCE_CM     = 20;    // stop/turn if something is closer than this
const unsigned long STOP_PAUSE_MS  = 400;   // "stop for a while" before turning
const unsigned long AVOID_TURN_MS  = 500;   // how long to turn when avoiding

const unsigned long SENSOR_READ_INTERVAL_MS = 2000;  // soil/light/DHT read rate (also history tick)
const unsigned long LCD_SWITCH_INTERVAL_MS  = 3000;  // how often LCD screen flips
const unsigned long DISTANCE_CHECK_INTERVAL_MS = 100; // obstacle check rate

const unsigned long SOIL_PROBE_INTERVAL_MS = 30000;  // how often to auto-dip the probe
const unsigned long SOIL_PROBE_DWELL_MS    = 2000;   // how long probe stays down

const int SOIL_RAW_DRY = 4095;
const int SOIL_RAW_WET = 1500;

const int LDR_RAW_DARK   = 300;
const int LDR_RAW_BRIGHT = 4095;

// Servo angles for the soil probe arm. The dashboard slider is capped to
// this same 0-90 range — if you change SERVO_DOWN_ANGLE, also update the
// slider's "max" attribute inside INDEX_HTML below.
const int SERVO_UP_ANGLE   = 0;    // resting / traveling position
const int SERVO_DOWN_ANGLE = 90;   // lowered into the soil

// How many sensor readings to keep for the dashboard history charts.
// At a 2s read interval, 30 samples = 1 minute of history.
#define HISTORY_SIZE 30

// ---------------------------------------------------------------------
// GLOBAL OBJECTS
// ---------------------------------------------------------------------
DHT dht(DHT_PIN, DHT_TYPE);
Servo probeServo;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// ---------------------------------------------------------------------
// LIVE SENSOR STATE
// ---------------------------------------------------------------------
float g_distanceCm   = -1;
int   g_soilPercent  = 0;
int   g_lightPercent = 0;
float g_temperatureC = 0;
float g_humidityPct  = 0;
bool  g_probeDown     = false;
bool  g_obstacleAvoiding = false;

bool g_running = true;
bool g_lastTurnWasRight = true;

bool g_manualServoActive = false;
int  g_currentServoAngle = SERVO_UP_ANGLE;

float g_histDistance[HISTORY_SIZE];
int   g_histSoil[HISTORY_SIZE];
int   g_histLight[HISTORY_SIZE];
float g_histTemp[HISTORY_SIZE];
float g_histHumidity[HISTORY_SIZE];
int   g_histIndex = 0;
int   g_histCount = 0;

unsigned long g_statusNavigatingMs = 0;
unsigned long g_statusAvoidingMs   = 0;
unsigned long g_statusStoppedMs    = 0;

unsigned long t_lastDistanceCheck = 0;
unsigned long t_lastSensorRead    = 0;
unsigned long t_lastLcdSwitch     = 0;
unsigned long t_lastSoilProbe     = 0;
unsigned long t_lastWifiCheck     = 0;
unsigned long t_lastStatusTick    = 0;
bool lcdShowingScreenA = true;

const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;

// ========================================================================
// MOTOR CONTROL
// ========================================================================
void motorsStop() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
}

void motorsForward() {
  digitalWrite(L298N_IN1, HIGH);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
}

void motorsBackward() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, HIGH);
}

void motorsTurnLeft() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
}

void motorsTurnRight() {
  digitalWrite(L298N_IN1, HIGH);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, HIGH);
}

// ========================================================================
// ULTRASONIC DISTANCE (HC-SR04)
// ========================================================================
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1;
  }
  return duration * 0.0343 / 2.0;
}

// ========================================================================
// OBSTACLE AVOIDANCE (Functionality 1)
// Drive forward continuously -> on obstacle, stop for a beat, turn
// (alternating right/left so it doesn't ping-pong in a corner), then
// go back to driving forward. Fully paused by the Start/Stop button.
// ========================================================================
void handleObstacleAvoidance() {
  if (!g_running) {
    motorsStop();
    g_obstacleAvoiding = false;
    return;
  }

  if (g_distanceCm > 0 && g_distanceCm < OBSTACLE_DISTANCE_CM) {
    g_obstacleAvoiding = true;

    motorsStop();
    delay(STOP_PAUSE_MS);

    if (g_lastTurnWasRight) {
      motorsTurnLeft();
    } else {
      motorsTurnRight();
    }
    g_lastTurnWasRight = !g_lastTurnWasRight;
    delay(AVOID_TURN_MS);

    motorsStop();
    delay(100);

    g_obstacleAvoiding = false;
  } else {
    motorsForward();
  }
}

// ========================================================================
// SOIL + LIGHT + DHT SENSING (Functionality 2)
// ========================================================================
void pushHistory() {
  g_histDistance[g_histIndex] = g_distanceCm;
  g_histSoil[g_histIndex]     = g_soilPercent;
  g_histLight[g_histIndex]    = g_lightPercent;
  g_histTemp[g_histIndex]     = g_temperatureC;
  g_histHumidity[g_histIndex] = g_humidityPct;

  g_histIndex = (g_histIndex + 1) % HISTORY_SIZE;
  if (g_histCount < HISTORY_SIZE) g_histCount++;
}

void readEnvironmentSensors() {
  int rawSoil = analogRead(SOIL_PIN);
  int rawLdr  = analogRead(LDR_PIN);

  g_soilPercent = map(rawSoil, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100);
  g_soilPercent = constrain(g_soilPercent, 0, 100);

  g_lightPercent = map(rawLdr, LDR_RAW_DARK, LDR_RAW_BRIGHT, 0, 100);
  g_lightPercent = constrain(g_lightPercent, 0, 100);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) g_humidityPct = h;
  if (!isnan(t)) g_temperatureC = t;

  pushHistory();
}

void handleSoilProbeCycle() {
  unsigned long now = millis();
  if (now - t_lastSoilProbe < SOIL_PROBE_INTERVAL_MS) return;
  t_lastSoilProbe = now;

  motorsStop();

  g_probeDown = true;
  g_currentServoAngle = SERVO_DOWN_ANGLE;
  probeServo.write(SERVO_DOWN_ANGLE);
  delay(SOIL_PROBE_DWELL_MS);

  int rawSoil = analogRead(SOIL_PIN);
  g_soilPercent = constrain(map(rawSoil, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100), 0, 100);

  probeServo.write(SERVO_UP_ANGLE);
  g_currentServoAngle = SERVO_UP_ANGLE;
  delay(500);
  g_probeDown = false;
}

// ========================================================================
// LCD DISPLAY
// ========================================================================
void updateLcd() {
  unsigned long now = millis();
  if (now - t_lastLcdSwitch < LCD_SWITCH_INTERVAL_MS) return;
  t_lastLcdSwitch = now;
  lcdShowingScreenA = !lcdShowingScreenA;

  lcd.clear();
  if (lcdShowingScreenA) {
    lcd.setCursor(0, 0);
    lcd.print("Dist:");
    if (g_distanceCm > 0) lcd.print(g_distanceCm, 0); else lcd.print("--");
    lcd.print("cm ");
    lcd.print(g_running ? (g_obstacleAvoiding ? "AVD" : "RUN") : "STP");

    lcd.setCursor(0, 1);
    lcd.print("Soil:");
    lcd.print(g_soilPercent);
    lcd.print("%");
    if (g_probeDown) lcd.print(" DOWN");
    else if (g_manualServoActive) lcd.print(" MAN");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Light:");
    lcd.print(g_lightPercent);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(g_temperatureC, 1);
    lcd.print("C H:");
    lcd.print(g_humidityPct, 0);
    lcd.print("%");
  }
}

// ========================================================================
// STATUS TIME TRACKING (feeds the dashboard pie chart)
// ========================================================================
void updateStatusTimers() {
  unsigned long now = millis();
  unsigned long delta = now - t_lastStatusTick;
  t_lastStatusTick = now;

  if (!g_running) {
    g_statusStoppedMs += delta;
  } else if (g_obstacleAvoiding) {
    g_statusAvoidingMs += delta;
  } else {
    g_statusNavigatingMs += delta;
  }
}

// ========================================================================
// JSON HELPERS for the history arrays
// ========================================================================
String intArrayToJson(int* arr, int count, int startIdx, int size) {
  String out = "[";
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % size;
    out += String(arr[idx]);
    if (i < count - 1) out += ",";
  }
  out += "]";
  return out;
}

String floatArrayToJson(float* arr, int count, int startIdx, int size) {
  String out = "[";
  for (int i = 0; i < count; i++) {
    int idx = (startIdx + i) % size;
    out += String(arr[idx], 1);
    if (i < count - 1) out += ",";
  }
  out += "]";
  return out;
}

// ========================================================================
// WEB DASHBOARD (Functionality 3)
// Served as a single static page from flash (PROGMEM) — loaded once,
// then updated live via a small JSON endpoint polled every 1.5s. This
// avoids rebuilding + re-sending a full HTML page (and a full browser
// reload) every refresh, which is both slower and heavier than this
// fetch-based approach.
// ========================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>Aestra Rover Dashboard</title>
<style>
  :root{--green:#2e8b57;--orange:#e8a93d;--red:#c0392b;--blue:#3b82c4;--bg:#f4f6f0;--card:#ffffff;--text:#2c3e2d;}
  *{box-sizing:border-box;}
  body{font-family:'Segoe UI',Arial,sans-serif;background:var(--bg);color:var(--text);margin:0;padding:20px;}
  h1{text-align:center;color:var(--green);margin-bottom:4px;}
  .subtitle{text-align:center;color:#888;font-size:13px;margin-bottom:20px;}
  .grid{display:flex;flex-wrap:wrap;gap:14px;justify-content:center;margin-bottom:24px;}
  .card{background:var(--card);border-radius:14px;box-shadow:0 2px 10px rgba(0,0,0,0.08);padding:16px 24px;min-width:120px;text-align:center;}
  .card .icon{font-size:22px;}
  .card .label{font-size:12px;color:#888;margin-top:4px;}
  .card .value{font-size:24px;font-weight:700;margin-top:2px;}
  .low{color:var(--red);} .mid{color:var(--orange);} .high{color:var(--green);}
  .controls{display:flex;flex-wrap:wrap;gap:20px;justify-content:center;align-items:center;background:var(--card);border-radius:14px;padding:16px;margin-bottom:24px;box-shadow:0 2px 10px rgba(0,0,0,0.08);}
  .btn{border:none;border-radius:10px;padding:12px 26px;font-size:16px;font-weight:700;color:#fff;cursor:pointer;}
  .btn-start{background:var(--green);}
  .btn-stop{background:var(--red);}
  .badge{display:inline-block;padding:5px 16px;border-radius:20px;color:#fff;font-size:13px;font-weight:700;}
  .badge-green{background:var(--green);} .badge-orange{background:var(--orange);} .badge-red{background:var(--red);}
  .slider-box{display:flex;flex-direction:column;align-items:center;}
  .slider-box label{font-size:13px;margin-bottom:4px;}
  input[type=range]{width:200px;}
  .charts{display:flex;flex-wrap:wrap;gap:16px;justify-content:center;}
  .chart-card{background:var(--card);border-radius:14px;box-shadow:0 2px 10px rgba(0,0,0,0.08);padding:16px;text-align:center;}
  .chart-card h3{margin:0 0 10px 0;font-size:14px;color:#555;}
  .legend{display:flex;gap:10px;justify-content:center;margin-top:8px;font-size:12px;flex-wrap:wrap;}
  .dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px;}
  footer{text-align:center;color:#aaa;font-size:11px;margin-top:24px;}
</style>
</head>
<body>
<h1>Aestra Rover</h1>
<div class='subtitle'>Live Telemetry &amp; Control Dashboard</div>

<div class='controls'>
  <span id='statusBadge' class='badge badge-green'>NAVIGATING</span>
  <button id='startStopBtn' class='btn btn-stop'>Stop</button>
  <div class='slider-box'>
    <label for='servoSlider'>Soil Probe Position: <span id='servoAngleLabel'>0&deg;</span></label>
    <input type='range' id='servoSlider' min='0' max='90' value='0'>
    <label style='font-size:12px;margin-top:6px;'><input type='checkbox' id='manualToggle'> Manual control</label>
  </div>
</div>

<div class='grid'>
  <div class='card'><div class='icon'>&#128207;</div><div class='value' id='distance'>--</div><div class='label'>Distance</div></div>
  <div class='card'><div class='icon'>&#127793;</div><div class='value' id='soil'>--</div><div class='label'>Soil Moisture</div></div>
  <div class='card'><div class='icon'>&#128161;</div><div class='value' id='light'>--</div><div class='label'>Ambient Light</div></div>
  <div class='card'><div class='icon'>&#127777;</div><div class='value' id='temp'>--</div><div class='label'>Air Temperature</div></div>
  <div class='card'><div class='icon'>&#128167;</div><div class='value' id='humidity'>--</div><div class='label'>Humidity</div></div>
  <div class='card'><div class='icon'>&#9995;</div><div class='value' id='probe'>--</div><div class='label'>Soil Probe</div></div>
</div>

<div class='charts'>
  <div class='chart-card'><h3>Soil Moisture History</h3><canvas id='chartLine' width='260' height='140'></canvas></div>
  <div class='chart-card'><h3>Current Snapshot</h3><canvas id='chartBar' width='260' height='140'></canvas></div>
  <div class='chart-card'><h3>Status Distribution</h3><canvas id='chartPie' width='140' height='140'></canvas>
    <div class='legend'>
      <span><span class='dot' style='background:#2e8b57'></span>Navigating</span>
      <span><span class='dot' style='background:#e8a93d'></span>Avoiding</span>
      <span><span class='dot' style='background:#c0392b'></span>Stopped</span>
    </div>
  </div>
</div>

<footer>Auto-updating every 1.5s &middot; Aestra Final Project</footer>

<script>
const $ = id => document.getElementById(id);
let runningState = true;
let manualDragging = false;

function colorize(id, val){
  const el = $(id);
  el.classList.remove('low','mid','high');
  if (val < 30) el.classList.add('low');
  else if (val < 70) el.classList.add('mid');
  else el.classList.add('high');
}

function drawLineChart(canvas, data, color){
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0,0,w,h);
  if (!data || data.length < 2) { ctx.fillStyle='#aaa'; ctx.font='12px Arial'; ctx.textAlign='center'; ctx.fillText('Gathering data...', w/2, h/2); return; }
  const max = Math.max(...data, 1);
  const min = Math.min(...data, 0);
  const range = (max - min) || 1;
  const stepX = w / (data.length - 1);
  ctx.beginPath();
  data.forEach((v,i)=>{
    const x = i*stepX;
    const y = h - ((v - min)/range)*(h-10) - 5;
    if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  });
  ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
  ctx.lineTo(w,h); ctx.lineTo(0,h); ctx.closePath();
  ctx.fillStyle = color + '33'; ctx.fill();
}

function drawBarChart(canvas, labels, data, colors){
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0,0,w,h);
  const max = 100;
  const gap = w/data.length;
  const barW = gap*0.5;
  data.forEach((v,i)=>{
    const barH = (Math.max(0,Math.min(v,100))/max)*(h-30);
    const x = i*gap + (gap-barW)/2;
    const y = h-barH-16;
    ctx.fillStyle = colors[i];
    ctx.fillRect(x,y,barW,barH);
    ctx.fillStyle = '#555';
    ctx.font='11px Arial';
    ctx.textAlign='center';
    ctx.fillText(labels[i], x+barW/2, h-3);
    ctx.fillText(v+'%', x+barW/2, y-4);
  });
}

function drawPieChart(canvas, data, colors){
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0,0,w,h);
  const total = data.reduce((a,b)=>a+b,0) || 1;
  const cx=w/2, cy=h/2, r=Math.min(w,h)/2-4;
  let start=-Math.PI/2;
  data.forEach((v,i)=>{
    const angle = (v/total)*Math.PI*2;
    ctx.beginPath();
    ctx.moveTo(cx,cy);
    ctx.arc(cx,cy,r,start,start+angle);
    ctx.closePath();
    ctx.fillStyle=colors[i];
    ctx.fill();
    start+=angle;
  });
}

async function refresh(){
  try {
    const res = await fetch('/api/data');
    const d = await res.json();
    runningState = d.running;

    $('distance').textContent = d.distance > 0 ? d.distance.toFixed(0)+' cm' : '--';
    $('soil').textContent = d.soil+'%';
    $('light').textContent = d.light+'%';
    $('temp').textContent = d.temp.toFixed(1)+' \u00b0C';
    $('humidity').textContent = d.humidity.toFixed(0)+'%';
    $('probe').textContent = d.probeDown ? 'LOWERED' : 'UP';

    colorize('soil', d.soil);
    colorize('light', d.light);
    colorize('humidity', d.humidity);

    const badge = $('statusBadge');
    if (!d.running) { badge.textContent='STOPPED'; badge.className='badge badge-red'; }
    else if (d.avoiding) { badge.textContent='AVOIDING'; badge.className='badge badge-orange'; }
    else { badge.textContent='NAVIGATING'; badge.className='badge badge-green'; }

    const btn = $('startStopBtn');
    btn.textContent = d.running ? 'Stop' : 'Start';
    btn.className = 'btn ' + (d.running ? 'btn-stop' : 'btn-start');

    if (!manualDragging) {
      $('servoSlider').value = d.servoAngle;
      $('servoAngleLabel').textContent = d.servoAngle + '\u00b0';
    }
    $('manualToggle').checked = d.manualServo;

    drawLineChart($('chartLine'), d.historySoil, '#2e8b57');
    drawBarChart($('chartBar'), ['Soil','Light','Humid'], [d.soil, d.light, Math.round(d.humidity)], ['#2e8b57','#e8a93d','#3b82c4']);
    drawPieChart($('chartPie'), [d.statusNavigating, d.statusAvoiding, d.statusStopped], ['#2e8b57','#e8a93d','#c0392b']);
  } catch (e) { /* network hiccup, next poll will retry */ }
}

$('startStopBtn').addEventListener('click', ()=>{
  fetch(runningState ? '/api/stop' : '/api/start').then(refresh);
});

const slider = $('servoSlider');
slider.addEventListener('input', ()=>{
  manualDragging = true;
  $('servoAngleLabel').textContent = slider.value + '\u00b0';
});
slider.addEventListener('change', ()=>{
  fetch('/api/servo?angle=' + slider.value).then(()=>{ manualDragging=false; refresh(); });
});

$('manualToggle').addEventListener('change', (e)=>{
  if (!e.target.checked) fetch('/api/servo_auto').then(refresh);
  else fetch('/api/servo?angle=' + slider.value).then(refresh);
});

setInterval(refresh, 1500);
refresh();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiData() {
  int startIdx = (g_histIndex - g_histCount + HISTORY_SIZE) % HISTORY_SIZE;

  String json = "{";
  json += "\"running\":" + String(g_running ? "true" : "false") + ",";
  json += "\"distance\":" + String(g_distanceCm, 1) + ",";
  json += "\"soil\":" + String(g_soilPercent) + ",";
  json += "\"light\":" + String(g_lightPercent) + ",";
  json += "\"temp\":" + String(g_temperatureC, 1) + ",";
  json += "\"humidity\":" + String(g_humidityPct, 1) + ",";
  json += "\"probeDown\":" + String(g_probeDown ? "true" : "false") + ",";
  json += "\"avoiding\":" + String(g_obstacleAvoiding ? "true" : "false") + ",";
  json += "\"manualServo\":" + String(g_manualServoActive ? "true" : "false") + ",";
  json += "\"servoAngle\":" + String(g_currentServoAngle) + ",";
  json += "\"statusNavigating\":" + String(g_statusNavigatingMs) + ",";
  json += "\"statusAvoiding\":" + String(g_statusAvoidingMs) + ",";
  json += "\"statusStopped\":" + String(g_statusStoppedMs) + ",";
  json += "\"historySoil\":" + intArrayToJson(g_histSoil, g_histCount, startIdx, HISTORY_SIZE) + ",";
  json += "\"historyLight\":" + intArrayToJson(g_histLight, g_histCount, startIdx, HISTORY_SIZE) + ",";
  json += "\"historyDistance\":" + floatArrayToJson(g_histDistance, g_histCount, startIdx, HISTORY_SIZE) + ",";
  json += "\"historyTemp\":" + floatArrayToJson(g_histTemp, g_histCount, startIdx, HISTORY_SIZE) + ",";
  json += "\"historyHumidity\":" + floatArrayToJson(g_histHumidity, g_histCount, startIdx, HISTORY_SIZE);
  json += "}";

  server.send(200, "application/json", json);
}

void handleServoSet() {
  if (server.hasArg("angle")) {
    int angle = server.arg("angle").toInt();
    angle = constrain(angle, SERVO_UP_ANGLE, SERVO_DOWN_ANGLE);
    g_manualServoActive = true;
    g_currentServoAngle = angle;
    probeServo.write(angle);
    g_probeDown = (angle > (SERVO_DOWN_ANGLE / 2));
  }
  server.send(200, "text/plain", "ok");
}

void handleServoAuto() {
  g_manualServoActive = false;
  g_currentServoAngle = SERVO_UP_ANGLE;
  g_probeDown = false;
  probeServo.write(SERVO_UP_ANGLE);
  t_lastSoilProbe = millis();
  server.send(200, "text/plain", "ok");
}

void handleStart() {
  g_running = true;
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  g_running = false;
  motorsStop();
  g_obstacleAvoiding = false;
  server.send(200, "text/plain", "ok");
}

// ========================================================================
// SETUP
// ========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);
  motorsStop();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(SOIL_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  dht.begin();

  probeServo.setPeriodHertz(50);
  probeServo.attach(SERVO_PIN, 500, 2400);
  probeServo.write(SERVO_UP_ANGLE);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Aestra Rover");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");

  WiFi.persistent(false);
  WiFi.setSleep(WIFI_PS_NONE);

  if (USE_ACCESS_POINT_MODE) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.print("Access Point started. Connect to Wi-Fi: ");
    Serial.println(AP_SSID);
    Serial.print("Dashboard IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(STA_SSID, STA_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
      delay(300);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected!");
      Serial.print("Dashboard IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nCould not connect — will keep retrying in loop().");
    }
  }

  server.on("/", handleRoot);
  server.on("/api/data", handleApiData);
  server.on("/api/start", handleStart);
  server.on("/api/stop", handleStop);
  server.on("/api/servo", handleServoSet);
  server.on("/api/servo_auto", handleServoAuto);
  server.begin();

  t_lastStatusTick = millis();

  delay(1000);
  lcd.clear();
}

// ========================================================================
// MAIN LOOP
// ========================================================================
void handleWifiReconnect() {
  if (USE_ACCESS_POINT_MODE) return;
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - t_lastWifiCheck < WIFI_RECONNECT_INTERVAL_MS) return;
  t_lastWifiCheck = now;

  Serial.println("Wi-Fi lost — reconnecting...");
  WiFi.reconnect();
}

void loop() {
  server.handleClient();
  handleWifiReconnect();
  updateStatusTimers();

  unsigned long now = millis();

  if (now - t_lastDistanceCheck >= DISTANCE_CHECK_INTERVAL_MS) {
    t_lastDistanceCheck = now;
    g_distanceCm = readDistanceCm();
    handleObstacleAvoidance();
  }

  if (now - t_lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    t_lastSensorRead = now;
    readEnvironmentSensors();
  }

  if (!g_manualServoActive) {
    handleSoilProbeCycle();
  }

  updateLcd();
}
