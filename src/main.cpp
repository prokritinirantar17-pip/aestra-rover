/*
  ========================================================================
  AESTRA — Autonomous Smart Agricultural Monitoring Rover
  ========================================================================
  Built for: ESP32 Dev Board (USB Type-C)

  Functionalities:
    1. Autonomous Navigation & Obstacle Avoidance (HC-SR04 + L298N motors)
       - AUTO mode: drives forward continuously, stops on obstacle, turns
         (alternating L/R), resumes. Fully non-blocking (see state
         machine notes below) so the web server never freezes.
    2. Intelligent Environmental & Soil Sensing (Soil moisture + LDR light
       sensor, with a servo that lowers a probe arm into the soil)
       - Automatic timer-based dipping, OR manual slider control from the
         dashboard. Also non-blocking.
    3. Wireless Telemetry & IoT Dashboard + MANUAL "Game Mode" driving
       - Immersive dark/neon dashboard: live gauges, history charts,
         mode switch (STOP / AUTO / MANUAL), and an on-screen joystick
         for direct driving with a collision-safety override.

  IMPORTANT ENGINEERING NOTE — why this version is non-blocking:
    The previous version used delay() inside the soil-probe and
    obstacle-avoidance routines. Any delay() call freezes the ENTIRE
    loop(), including server.handleClient() — so while the probe was
    dipping (~2.5s), every button press / slider drag on the dashboard
    appeared to "hang". This version replaces both routines with
    millis()-based state machines that never call delay(), so the
    server stays responsive at all times, even mid-maneuver.

  NOT implemented (excluded from the final build):
    - Buzzer, Push Buttons, Potentiometer, manual/auto toggle switch

  ------------------------------------------------------------------------
  LIBRARIES TO INSTALL (Arduino Library Manager, or PlatformIO lib_deps):
    - "ESP32Servo"                by Kevin Harrington / Jonathan Nordbrook
    - "LiquidCrystal I2C"         by Frank de Brabander
    - "DHT sensor library"        by Adafruit
    - "Adafruit Unified Sensor"   by Adafruit   (dependency of the above)
    (WiFi.h and WebServer.h are already built into the ESP32 core)

  ------------------------------------------------------------------------
  PIN MAP:
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
    ENA / ENB on the L298N are left jumper-capped ON (always enabled) —
    motor speed is fixed, IN1..IN4 only control direction.
  ========================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------------------------------------------------------------------
// WI-FI CONFIG
// ---------------------------------------------------------------------
#define USE_ACCESS_POINT_MODE true

const char* AP_SSID     = "Aestra-Rover";
const char* AP_PASSWORD = "aestra123";

const char* STA_SSID     = "Think Tank _ 04";
const char* STA_PASSWORD = "thinktank14312";

WebServer server(80);

// ---------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------
#define DHT_PIN        4
#define DHT_TYPE       DHT11
#define SERVO_PIN      5
#define L298N_IN1      25
#define L298N_IN2      26
#define L298N_IN3      27
#define L298N_IN4      14
#define TRIG_PIN       18
#define ECHO_PIN       19
#define SOIL_PIN       34
#define LDR_PIN        35
#define LCD_ADDRESS    0x27
#define LCD_COLS       16
#define LCD_ROWS       2

// ---------------------------------------------------------------------
// TUNABLE CONSTANTS
// ---------------------------------------------------------------------
const int OBSTACLE_DISTANCE_CM     = 20;
const unsigned long STOP_PAUSE_MS  = 400;
const unsigned long AVOID_TURN_MS  = 500;
const unsigned long AVOID_SETTLE_MS = 100;

const unsigned long SENSOR_READ_INTERVAL_MS = 2000;
const unsigned long LCD_SWITCH_INTERVAL_MS  = 3000;
const unsigned long DISTANCE_CHECK_INTERVAL_MS = 100;

const unsigned long SOIL_PROBE_INTERVAL_MS = 30000;
const unsigned long SOIL_PROBE_DWELL_MS    = 2000;
const unsigned long SOIL_PROBE_RAISE_MS    = 500;

const int SOIL_RAW_DRY = 4095;
const int SOIL_RAW_WET = 1500;
const int LDR_RAW_DARK   = 300;
const int LDR_RAW_BRIGHT = 4095;

const int SERVO_UP_ANGLE   = 0;
const int SERVO_DOWN_ANGLE = 90;

// Manual drive (Game Mode) safety + reliability
const int SAFETY_STOP_CM = 8;                     // never drive forward closer than this
const unsigned long DRIVE_WATCHDOG_MS = 800;       // auto-stop if joystick signal is lost

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

// Modes: rover starts fully STOPPED until the dashboard says otherwise.
bool g_running = false;            // AUTO mode
bool g_manualDriveActive = false;  // MANUAL / Game Mode
bool g_lastTurnWasRight = true;

// Manual drive (joystick) state
String g_currentDriveCmd = "STOP";
bool   g_safetyBlocked = false;
unsigned long t_lastDriveCmd = 0;

// Manual servo (soil probe slider) state
bool g_manualServoActive = false;
int  g_currentServoAngle = SERVO_UP_ANGLE;

// History buffers
float g_histDistance[HISTORY_SIZE];
int   g_histSoil[HISTORY_SIZE];
int   g_histLight[HISTORY_SIZE];
float g_histTemp[HISTORY_SIZE];
float g_histHumidity[HISTORY_SIZE];
int   g_histIndex = 0;
int   g_histCount = 0;

// Status time accumulators (pie chart) + session stats
unsigned long g_statusNavigatingMs = 0;
unsigned long g_statusAvoidingMs   = 0;
unsigned long g_statusManualMs     = 0;
unsigned long g_statusStoppedMs    = 0;
unsigned long g_obstacleAvoidCount = 0;
unsigned long g_probeDipCount      = 0;

// Timers
unsigned long t_lastDistanceCheck = 0;
unsigned long t_lastSensorRead    = 0;
unsigned long t_lastLcdSwitch     = 0;
unsigned long t_lastSoilProbe     = 0;
unsigned long t_lastWifiCheck     = 0;
unsigned long t_lastStatusTick    = 0;
bool lcdShowingScreenA = true;

const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;

// Non-blocking state machines
enum AvoidState { AVOID_IDLE, AVOID_PAUSE, AVOID_TURN, AVOID_SETTLE };
AvoidState avoidState = AVOID_IDLE;
unsigned long avoidStateStart = 0;

enum ProbeState { PROBE_IDLE, PROBE_LOWERING, PROBE_DWELL, PROBE_RAISING };
ProbeState probeState = PROBE_IDLE;
unsigned long probeStateStart = 0;

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
  if (duration == 0) return -1;
  return duration * 0.0343 / 2.0;
}

// ========================================================================
// OBSTACLE AVOIDANCE — NON-BLOCKING STATE MACHINE (AUTO mode only)
// ========================================================================
void updateObstacleAvoidance() {
  if (g_manualDriveActive) {
    avoidState = AVOID_IDLE;
    g_obstacleAvoiding = false;
    return; // the joystick handler owns the motors in this mode
  }
  if (!g_running) {
    motorsStop();
    avoidState = AVOID_IDLE;
    g_obstacleAvoiding = false;
    return;
  }

  unsigned long now = millis();
  switch (avoidState) {
    case AVOID_IDLE:
      if (g_distanceCm > 0 && g_distanceCm < OBSTACLE_DISTANCE_CM) {
        g_obstacleAvoiding = true;
        g_obstacleAvoidCount++;
        motorsStop();
        avoidState = AVOID_PAUSE;
        avoidStateStart = now;
      } else {
        motorsForward();
      }
      break;

    case AVOID_PAUSE:
      if (now - avoidStateStart >= STOP_PAUSE_MS) {
        if (g_lastTurnWasRight) motorsTurnLeft(); else motorsTurnRight();
        g_lastTurnWasRight = !g_lastTurnWasRight;
        avoidState = AVOID_TURN;
        avoidStateStart = now;
      }
      break;

    case AVOID_TURN:
      if (now - avoidStateStart >= AVOID_TURN_MS) {
        motorsStop();
        avoidState = AVOID_SETTLE;
        avoidStateStart = now;
      }
      break;

    case AVOID_SETTLE:
      if (now - avoidStateStart >= AVOID_SETTLE_MS) {
        g_obstacleAvoiding = false;
        avoidState = AVOID_IDLE;
      }
      break;
  }
}

// ========================================================================
// MANUAL DRIVE (Game Mode / joystick)
// ========================================================================
void applyDriveCommand(const String& cmd) {
  t_lastDriveCmd = millis();
  g_safetyBlocked = false;

  if (cmd == "FWD") {
    if (g_distanceCm > 0 && g_distanceCm < SAFETY_STOP_CM) {
      motorsStop();
      g_safetyBlocked = true;
      g_currentDriveCmd = "STOP";
      return;
    }
    motorsForward();
  } else if (cmd == "BACK") {
    motorsBackward();
  } else if (cmd == "LEFT") {
    motorsTurnLeft();
  } else if (cmd == "RIGHT") {
    motorsTurnRight();
  } else {
    motorsStop();
  }
  g_currentDriveCmd = cmd;
}

void updateDriveWatchdog() {
  if (!g_manualDriveActive) return;
  if (millis() - t_lastDriveCmd > DRIVE_WATCHDOG_MS) {
    motorsStop();
    g_currentDriveCmd = "STOP";
  }
}

// ========================================================================
// SOIL + LIGHT + DHT SENSING
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

  g_soilPercent = constrain(map(rawSoil, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100), 0, 100);
  g_lightPercent = constrain(map(rawLdr, LDR_RAW_DARK, LDR_RAW_BRIGHT, 0, 100), 0, 100);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h)) g_humidityPct = h;
  if (!isnan(t)) g_temperatureC = t;

  pushHistory();
}

// Non-blocking soil probe cycle. Skipped while manual servo control is
// active. While lowering/dwelling/raising, it forcibly stops the drive
// motors each loop tick (so the arm isn't dragged along the ground).
void updateSoilProbeCycle() {
  if (g_manualServoActive) {
    probeState = PROBE_IDLE;
    return;
  }

  unsigned long now = millis();

  switch (probeState) {
    case PROBE_IDLE:
      if (now - t_lastSoilProbe >= SOIL_PROBE_INTERVAL_MS) {
        t_lastSoilProbe = now;
        g_probeDown = true;
        g_probeDipCount++;
        g_currentServoAngle = SERVO_DOWN_ANGLE;
        probeServo.write(SERVO_DOWN_ANGLE);
        probeState = PROBE_DWELL;
        probeStateStart = now;
      }
      break;

    case PROBE_DWELL:
      motorsStop();
      if (now - probeStateStart >= SOIL_PROBE_DWELL_MS) {
        int rawSoil = analogRead(SOIL_PIN);
        g_soilPercent = constrain(map(rawSoil, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100), 0, 100);
        probeServo.write(SERVO_UP_ANGLE);
        g_currentServoAngle = SERVO_UP_ANGLE;
        probeState = PROBE_RAISING;
        probeStateStart = now;
      }
      break;

    case PROBE_RAISING:
      motorsStop();
      if (now - probeStateStart >= SOIL_PROBE_RAISE_MS) {
        g_probeDown = false;
        probeState = PROBE_IDLE;
      }
      break;

    case PROBE_IDLE:
    default:
      break;
  }
}

// ========================================================================
// LCD DISPLAY
// ========================================================================
void updateLcd() {
  unsigned long now = millis();
  if (now - t_lastLcdSwitch < LCD_SWITCH_INTERVAL_MS) return;
  t_lastLcdSwitch = now;
  lcdShowingScreenA = !lcdShowingScreenA;

  String modeTag = g_manualDriveActive ? "MAN" : (g_running ? (g_obstacleAvoiding ? "AVD" : "RUN") : "STP");

  lcd.clear();
  if (lcdShowingScreenA) {
    lcd.setCursor(0, 0);
    lcd.print("Dist:");
    if (g_distanceCm > 0) lcd.print(g_distanceCm, 0); else lcd.print("--");
    lcd.print("cm ");
    lcd.print(modeTag);

    lcd.setCursor(0, 1);
    lcd.print("Soil:");
    lcd.print(g_soilPercent);
    lcd.print("%");
    if (g_probeDown) lcd.print(" DN");
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
// STATUS TIME TRACKING (feeds the pie chart + session stats)
// ========================================================================
void updateStatusTimers() {
  unsigned long now = millis();
  unsigned long delta = now - t_lastStatusTick;
  t_lastStatusTick = now;

  if (g_manualDriveActive) {
    g_statusManualMs += delta;
  } else if (!g_running) {
    g_statusStoppedMs += delta;
  } else if (g_obstacleAvoiding) {
    g_statusAvoidingMs += delta;
  } else {
    g_statusNavigatingMs += delta;
  }
}

// ========================================================================
// JSON HELPERS
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
// WEB DASHBOARD — immersive dark / neon theme, served from flash
// (PROGMEM), updated live via JSON polling. No external fonts/scripts,
// so it works fully offline in Access Point mode.
// ========================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<title>Aestra Rover</title>
<style>
  :root{
    --bg:#070910; --panel:rgba(255,255,255,0.045); --panel-border:rgba(255,255,255,0.09);
    --text:#e7ecf5; --muted:#8b93a7;
    --green:#39ff88; --pink:#ff3dd4; --cyan:#39e6ff; --orange:#ffb23d; --red:#ff4d5e; --blue:#4da3ff;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; padding:20px; min-height:100vh;
    font-family:'Segoe UI',system-ui,Arial,sans-serif; color:var(--text);
    background:
      radial-gradient(circle at 15% 0%, rgba(57,230,255,0.10), transparent 40%),
      radial-gradient(circle at 85% 10%, rgba(255,61,212,0.09), transparent 45%),
      var(--bg);
  }
  h1{margin:0; font-size:22px; letter-spacing:2px;}
  .topbar{display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:10px; margin-bottom:18px;}
  .brand{display:flex; align-items:center; gap:10px;}
  .brand .tag{font-size:11px; color:var(--muted); letter-spacing:1px;}
  .live{display:flex; align-items:center; gap:8px; font-size:12px; color:var(--muted);}
  .live-dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:dotPulse 1.6s infinite;}
  .live-dot.offline{background:var(--red);box-shadow:0 0 8px var(--red);animation:none;}
  @keyframes dotPulse{0%,100%{opacity:1;}50%{opacity:0.3;}}

  .panel{background:var(--panel); border:1px solid var(--panel-border); border-radius:16px; backdrop-filter:blur(10px); padding:16px;}

  .mode-row{display:flex; flex-wrap:wrap; gap:12px; align-items:center; justify-content:center; margin-bottom:18px;}
  .mode-switch{display:flex; gap:8px; background:rgba(255,255,255,0.03); border:1px solid var(--panel-border); border-radius:999px; padding:5px;}
  .mode-btn{border:none; background:transparent; color:var(--muted); font-weight:700; font-size:12px; letter-spacing:1px; padding:10px 18px; border-radius:999px; cursor:pointer; transition:all .2s;}
  .mode-btn.active.stop{background:var(--red); color:#1a0505; box-shadow:0 0 14px rgba(255,77,94,0.6);}
  .mode-btn.active.auto{background:var(--green); color:#04140a; box-shadow:0 0 14px rgba(57,255,136,0.6);}
  .mode-btn.active.manual{background:var(--pink); color:#1a0414; box-shadow:0 0 14px rgba(255,61,212,0.6);}
  .status-badge{padding:6px 16px; border-radius:999px; font-size:12px; font-weight:700; letter-spacing:1px;}
  .status-badge.navigating{background:rgba(57,255,136,0.15); color:var(--green); border:1px solid var(--green);}
  .status-badge.avoiding{background:rgba(255,178,61,0.15); color:var(--orange); border:1px solid var(--orange);}
  .status-badge.stopped{background:rgba(255,77,94,0.15); color:var(--red); border:1px solid var(--red);}
  .status-badge.manual{background:rgba(255,61,212,0.15); color:var(--pink); border:1px solid var(--pink);}

  .game-panel{max-width:420px; margin:0 auto 22px auto; text-align:center; display:none;}
  .game-panel.show{display:block;}
  .game-title{font-size:20px; font-weight:800; letter-spacing:3px; color:var(--text); margin-bottom:4px;}
  .game-title .on-tag{color:var(--pink); animation:neonPulse 1.3s ease-in-out infinite;}
  @keyframes neonPulse{
    0%,100%{text-shadow:0 0 6px var(--pink),0 0 14px var(--pink),0 0 26px var(--pink);}
    50%{text-shadow:0 0 14px var(--pink),0 0 30px var(--pink),0 0 48px var(--pink);}
  }
  .game-hint{font-size:11px; color:var(--muted); margin-top:10px;}
  .safety-warn{display:none; background:rgba(255,77,94,0.15); border:1px solid var(--red); color:var(--red); font-weight:700; font-size:12px; letter-spacing:1px; padding:8px 12px; border-radius:10px; margin:10px 0; animation:warnPulse 1s infinite;}
  .safety-warn.show{display:block;}
  @keyframes warnPulse{0%,100%{opacity:1;}50%{opacity:0.5;}}

  .joystick-zone{position:relative; width:200px; height:200px; margin:16px auto; border-radius:50%; background:radial-gradient(circle,rgba(255,61,212,0.08),rgba(57,230,255,0.04)); border:1px solid rgba(255,61,212,0.35); box-shadow:0 0 30px rgba(255,61,212,0.15), inset 0 0 20px rgba(57,230,255,0.08); touch-action:none;}
  .joystick-ring{position:absolute; inset:14px; border-radius:50%; border:1px dashed rgba(255,255,255,0.15);}
  .joystick-knob{position:absolute; top:50%; left:50%; width:62px; height:62px; margin:-31px 0 0 -31px; border-radius:50%; background:radial-gradient(circle at 35% 30%,#ffffff,#ff3dd4 60%,#9b1d80); box-shadow:0 0 20px var(--pink), 0 0 40px rgba(255,61,212,0.4); cursor:grab;}

  .grid{display:flex; flex-wrap:wrap; gap:14px; justify-content:center; margin-bottom:20px;}
  .gauge-wrap{position:relative; width:140px; text-align:center;}
  .gauge-value{position:absolute; top:52%; left:50%; transform:translate(-50%,-50%); font-family:Consolas,monospace; font-size:20px; font-weight:700;}
  .gauge-label{margin-top:4px; font-size:11px; letter-spacing:1px; color:var(--muted); text-transform:uppercase;}

  .sub-panel{max-width:360px; margin:0 auto 20px auto;}
  .sub-panel h3{margin:0 0 10px 0; font-size:13px; letter-spacing:1px; color:var(--muted); text-transform:uppercase; text-align:center;}
  .slider-row{display:flex; flex-direction:column; align-items:center; gap:8px;}
  input[type=range]{width:220px; accent-color:var(--cyan);}
  .toggle-row{font-size:12px; color:var(--muted); display:flex; align-items:center; gap:6px;}

  .charts{display:flex; flex-wrap:wrap; gap:14px; justify-content:center; margin-bottom:20px;}
  .chart-card{text-align:center; padding:14px;}
  .chart-card h3{margin:0 0 10px 0; font-size:12px; letter-spacing:1px; color:var(--muted); text-transform:uppercase;}
  .legend{display:flex; gap:10px; justify-content:center; margin-top:8px; font-size:11px; color:var(--muted); flex-wrap:wrap;}
  .dot{display:inline-block; width:9px; height:9px; border-radius:50%; margin-right:4px;}

  .stats-row{display:flex; flex-wrap:wrap; gap:14px; justify-content:center; margin-bottom:10px;}
  .stat{text-align:center; min-width:100px;}
  .stat .stat-value{font-family:Consolas,monospace; font-size:18px; font-weight:700; color:var(--cyan);}
  .stat .stat-label{font-size:10px; color:var(--muted); letter-spacing:1px; text-transform:uppercase; margin-top:2px;}

  footer{text-align:center; color:#555c6e; font-size:11px; margin-top:16px;}
</style>
</head>
<body>

<div class='topbar'>
  <div class='brand'>
    <h1>AESTRA</h1>
    <div class='tag'>AGRI-ROVER CONTROL</div>
  </div>
  <div class='live'><span class='live-dot' id='liveDot'></span><span id='liveText'>Live</span></div>
</div>

<div class='mode-row'>
  <span id='statusBadge' class='status-badge stopped'>STOPPED</span>
  <div class='mode-switch'>
    <button class='mode-btn' id='btnStop'>STOP</button>
    <button class='mode-btn' id='btnAuto'>AUTO</button>
    <button class='mode-btn' id='btnManual'>MANUAL</button>
  </div>
</div>

<div class='game-panel panel' id='gamePanel'>
  <div class='game-title'>GAME MODE <span class='on-tag'>ON</span></div>
  <div class='safety-warn' id='safetyWarn'>OBSTACLE AHEAD — FORWARD BLOCKED</div>
  <div class='joystick-zone' id='joystickZone'>
    <div class='joystick-ring'></div>
    <div class='joystick-knob' id='joystickKnob'></div>
  </div>
  <div class='game-hint'>Drag the stick to drive &middot; release to stop</div>
</div>

<div class='grid'>
  <div class='gauge-wrap'><canvas id='gDistance' width='140' height='140'></canvas><div class='gauge-value' id='vDistance'>--</div><div class='gauge-label'>Distance</div></div>
  <div class='gauge-wrap'><canvas id='gSoil' width='140' height='140'></canvas><div class='gauge-value' id='vSoil'>--</div><div class='gauge-label'>Soil Moisture</div></div>
  <div class='gauge-wrap'><canvas id='gLight' width='140' height='140'></canvas><div class='gauge-value' id='vLight'>--</div><div class='gauge-label'>Ambient Light</div></div>
  <div class='gauge-wrap'><canvas id='gTemp' width='140' height='140'></canvas><div class='gauge-value' id='vTemp'>--</div><div class='gauge-label'>Temperature</div></div>
  <div class='gauge-wrap'><canvas id='gHumidity' width='140' height='140'></canvas><div class='gauge-value' id='vHumidity'>--</div><div class='gauge-label'>Humidity</div></div>
</div>

<div class='sub-panel panel'>
  <h3>Soil Probe Control</h3>
  <div class='slider-row'>
    <div>Position: <span id='servoAngleLabel'>0&deg;</span> &middot; <span id='probeStateLabel'>UP</span></div>
    <input type='range' id='servoSlider' min='0' max='90' value='0'>
    <label class='toggle-row'><input type='checkbox' id='manualToggle'> Manual probe control</label>
  </div>
</div>

<div class='charts'>
  <div class='chart-card panel'><h3>Soil Moisture History</h3><canvas id='chartLine' width='260' height='130'></canvas></div>
  <div class='chart-card panel'><h3>Current Snapshot</h3><canvas id='chartBar' width='260' height='130'></canvas></div>
  <div class='chart-card panel'><h3>Mode Distribution</h3><canvas id='chartPie' width='130' height='130'></canvas>
    <div class='legend'>
      <span><span class='dot' style='background:#39ff88'></span>Navigating</span>
      <span><span class='dot' style='background:#ffb23d'></span>Avoiding</span>
      <span><span class='dot' style='background:#ff3dd4'></span>Manual</span>
      <span><span class='dot' style='background:#ff4d5e'></span>Stopped</span>
    </div>
  </div>
</div>

<div class='stats-row panel'>
  <div class='stat'><div class='stat-value' id='statUptime'>00:00:00</div><div class='stat-label'>Uptime</div></div>
  <div class='stat'><div class='stat-value' id='statAvoided'>0</div><div class='stat-label'>Obstacles Avoided</div></div>
  <div class='stat'><div class='stat-value' id='statDips'>0</div><div class='stat-label'>Soil Checks</div></div>
</div>

<footer>Aestra Final Project &middot; fully offline-capable dashboard</footer>

<script>
const $ = id => document.getElementById(id);
let currentMode = 'STOPPED';
let manualDragging = false;
let lastUpdate = Date.now();

function colorFor(id){
  return {gDistance:'#39e6ff', gSoil:'#39ff88', gLight:'#ffb23d', gTemp:'#ff4d5e', gHumidity:'#4da3ff'}[id];
}

function drawGauge(canvas, value, min, max, color){
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0,0,w,h);
  const cx=w/2, cy=h/2, r=Math.min(w,h)/2-12;
  const startA = Math.PI*0.75, endA = Math.PI*2.25;
  const pct = Math.max(0, Math.min(1, (value-min)/(max-min)));
  const valA = startA + (endA-startA)*pct;

  ctx.beginPath(); ctx.arc(cx,cy,r,startA,endA);
  ctx.strokeStyle='rgba(255,255,255,0.08)'; ctx.lineWidth=10; ctx.lineCap='round'; ctx.stroke();

  ctx.beginPath(); ctx.arc(cx,cy,r,startA,valA);
  ctx.strokeStyle=color; ctx.lineWidth=10; ctx.lineCap='round';
  ctx.shadowColor=color; ctx.shadowBlur=14; ctx.stroke(); ctx.shadowBlur=0;
}

function drawLineChart(canvas, data, color){
  const ctx = canvas.getContext('2d'); const w=canvas.width, h=canvas.height;
  ctx.clearRect(0,0,w,h);
  if (!data || data.length<2){ ctx.fillStyle='#555c6e'; ctx.font='11px Arial'; ctx.textAlign='center'; ctx.fillText('Gathering data...', w/2, h/2); return; }
  const max=Math.max(...data,1), min=Math.min(...data,0), range=(max-min)||1;
  const stepX=w/(data.length-1);
  ctx.beginPath();
  data.forEach((v,i)=>{ const x=i*stepX, y=h-((v-min)/range)*(h-10)-5; i===0?ctx.moveTo(x,y):ctx.lineTo(x,y); });
  ctx.strokeStyle=color; ctx.lineWidth=2; ctx.shadowColor=color; ctx.shadowBlur=6; ctx.stroke(); ctx.shadowBlur=0;
  ctx.lineTo(w,h); ctx.lineTo(0,h); ctx.closePath();
  ctx.fillStyle=color+'22'; ctx.fill();
}

function drawBarChart(canvas, labels, data, colors){
  const ctx=canvas.getContext('2d'); const w=canvas.width, h=canvas.height;
  ctx.clearRect(0,0,w,h);
  const gap=w/data.length, barW=gap*0.5;
  data.forEach((v,i)=>{
    const barH=(Math.max(0,Math.min(v,100))/100)*(h-30);
    const x=i*gap+(gap-barW)/2, y=h-barH-16;
    ctx.fillStyle=colors[i]; ctx.shadowColor=colors[i]; ctx.shadowBlur=8;
    ctx.fillRect(x,y,barW,barH); ctx.shadowBlur=0;
    ctx.fillStyle='#c6ccda'; ctx.font='10px Arial'; ctx.textAlign='center';
    ctx.fillText(labels[i], x+barW/2, h-3);
    ctx.fillText(v+'%', x+barW/2, y-4);
  });
}

function drawPieChart(canvas, data, colors){
  const ctx=canvas.getContext('2d'); const w=canvas.width, h=canvas.height;
  ctx.clearRect(0,0,w,h);
  const total=data.reduce((a,b)=>a+b,0)||1;
  const cx=w/2, cy=h/2, r=Math.min(w,h)/2-4;
  let start=-Math.PI/2;
  data.forEach((v,i)=>{
    const angle=(v/total)*Math.PI*2;
    ctx.beginPath(); ctx.moveTo(cx,cy); ctx.arc(cx,cy,r,start,start+angle); ctx.closePath();
    ctx.fillStyle=colors[i]; ctx.fill();
    start+=angle;
  });
}

function formatUptime(ms){
  let s=Math.floor(ms/1000);
  const h=Math.floor(s/3600); s%=3600;
  const m=Math.floor(s/60); s%=60;
  return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');
}

function setMode(mode){
  currentMode = mode;
  ['btnStop','btnAuto','btnManual'].forEach(id=>$(id).className='mode-btn');
  if (mode==='STOPPED') $('btnStop').className='mode-btn active stop';
  if (mode==='AUTO') $('btnAuto').className='mode-btn active auto';
  if (mode==='MANUAL') $('btnManual').className='mode-btn active manual';
  $('gamePanel').className = 'game-panel panel' + (mode==='MANUAL' ? ' show' : '');
}

async function refresh(){
  try {
    const res = await fetch('/api/data');
    const d = await res.json();
    lastUpdate = Date.now();
    $('liveDot').className='live-dot'; $('liveText').textContent='Live';

    let mode = d.manualDrive ? 'MANUAL' : (d.running ? 'AUTO' : 'STOPPED');
    setMode(mode);

    const badge=$('statusBadge');
    if (mode==='MANUAL'){ badge.textContent='GAME MODE'; badge.className='status-badge manual'; }
    else if (mode==='STOPPED'){ badge.textContent='STOPPED'; badge.className='status-badge stopped'; }
    else if (d.avoiding){ badge.textContent='AVOIDING'; badge.className='status-badge avoiding'; }
    else { badge.textContent='NAVIGATING'; badge.className='status-badge navigating'; }

    $('safetyWarn').className='safety-warn' + (d.safetyBlocked ? ' show' : '');

    $('vDistance').textContent = d.distance>0 ? d.distance.toFixed(0)+'cm' : '--';
    $('vSoil').textContent = d.soil+'%';
    $('vLight').textContent = d.light+'%';
    $('vTemp').textContent = d.temp.toFixed(1)+'\u00b0';
    $('vHumidity').textContent = d.humidity.toFixed(0)+'%';

    drawGauge($('gDistance'), d.distance>0?d.distance:0, 0, 200, colorFor('gDistance'));
    drawGauge($('gSoil'), d.soil, 0, 100, colorFor('gSoil'));
    drawGauge($('gLight'), d.light, 0, 100, colorFor('gLight'));
    drawGauge($('gTemp'), d.temp, 0, 50, colorFor('gTemp'));
    drawGauge($('gHumidity'), d.humidity, 0, 100, colorFor('gHumidity'));

    $('probeStateLabel').textContent = d.probeDown ? 'DOWN' : 'UP';
    if (!manualDragging) {
      $('servoSlider').value = d.servoAngle;
      $('servoAngleLabel').textContent = d.servoAngle + '\u00b0';
    }
    $('manualToggle').checked = d.manualServo;

    drawLineChart($('chartLine'), d.historySoil, '#39ff88');
    drawBarChart($('chartBar'), ['Soil','Light','Humid'], [d.soil, d.light, Math.round(d.humidity)], ['#39ff88','#ffb23d','#4da3ff']);
    drawPieChart($('chartPie'), [d.statusNavigating, d.statusAvoiding, d.statusManual, d.statusStopped], ['#39ff88','#ffb23d','#ff3dd4','#ff4d5e']);

    $('statUptime').textContent = formatUptime(d.uptimeMs);
    $('statAvoided').textContent = d.obstaclesAvoided;
    $('statDips').textContent = d.probeDips;
  } catch (e) {
    if (Date.now()-lastUpdate > 4000){ $('liveDot').className='live-dot offline'; $('liveText').textContent='Offline'; }
  }
}

setInterval(()=>{ if (Date.now()-lastUpdate > 4000){ $('liveDot').className='live-dot offline'; $('liveText').textContent='Offline'; } }, 1000);

$('btnStop').addEventListener('click', ()=> fetch('/api/stop').then(refresh));
$('btnAuto').addEventListener('click', ()=> fetch('/api/start').then(refresh));
$('btnManual').addEventListener('click', ()=>{
  if (currentMode==='MANUAL') fetch('/api/manual_off').then(refresh);
  else fetch('/api/manual_on').then(refresh);
});

const slider = $('servoSlider');
let lastServoSend = 0;
slider.addEventListener('input', ()=>{
  manualDragging = true;
  $('servoAngleLabel').textContent = slider.value + '\u00b0';
  const now = Date.now();
  if (now - lastServoSend > 90) {
    lastServoSend = now;
    fetch('/api/servo?angle=' + slider.value).catch(()=>{});
  }
});
slider.addEventListener('change', ()=>{
  fetch('/api/servo?angle=' + slider.value).then(()=>{ manualDragging=false; refresh(); });
});

$('manualToggle').addEventListener('change', (e)=>{
  if (!e.target.checked) fetch('/api/servo_auto').then(refresh);
  else fetch('/api/servo?angle=' + slider.value).then(refresh);
});

// Joystick (pointer events = mouse + touch unified)
const zone = $('joystickZone'), knob = $('joystickKnob');
let dragging=false, lastCmd=null, lastSend=0;

function zoneGeo(){ const r=zone.getBoundingClientRect(); return {x:r.left+r.width/2, y:r.top+r.height/2, r:r.width/2}; }

function sendDrive(cmd){
  const now=Date.now();
  if (cmd===lastCmd && now-lastSend<300) return;
  lastCmd=cmd; lastSend=now;
  fetch('/api/drive?cmd='+cmd).catch(()=>{});
}

function handleMove(cx, cy){
  const c = zoneGeo();
  let dx = cx-c.x, dy = cy-c.y;
  const dist = Math.min(Math.hypot(dx,dy), c.r-20);
  const angle = Math.atan2(dy,dx);
  knob.style.transform = `translate(${Math.cos(angle)*dist}px, ${Math.sin(angle)*dist}px)`;

  const deadzone = (c.r-20)*0.3;
  let cmd='STOP';
  if (dist > deadzone) {
    cmd = Math.abs(dx) > Math.abs(dy) ? (dx>0?'RIGHT':'LEFT') : (dy>0?'BACK':'FWD');
  }
  sendDrive(cmd);
}

function resetKnob(){ knob.style.transform='translate(0px,0px)'; sendDrive('STOP'); lastCmd=null; }

zone.addEventListener('pointerdown', (e)=>{ dragging=true; zone.setPointerCapture(e.pointerId); handleMove(e.clientX,e.clientY); });
zone.addEventListener('pointermove', (e)=>{ if(dragging) handleMove(e.clientX,e.clientY); });
zone.addEventListener('pointerup', resetKnob);
zone.addEventListener('pointercancel', resetKnob);

setInterval(refresh, 1200);
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
  json += "\"manualDrive\":" + String(g_manualDriveActive ? "true" : "false") + ",";
  json += "\"safetyBlocked\":" + String(g_safetyBlocked ? "true" : "false") + ",";
  json += "\"driveCmd\":\"" + g_currentDriveCmd + "\",";
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
  json += "\"statusManual\":" + String(g_statusManualMs) + ",";
  json += "\"statusStopped\":" + String(g_statusStoppedMs) + ",";
  json += "\"obstaclesAvoided\":" + String(g_obstacleAvoidCount) + ",";
  json += "\"probeDips\":" + String(g_probeDipCount) + ",";
  json += "\"uptimeMs\":" + String(millis()) + ",";
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
    bool wasDown = g_probeDown;
    g_manualServoActive = true;
    g_currentServoAngle = angle;
    probeServo.write(angle);                 // applied immediately — no blocking delay anywhere in this path
    g_probeDown = (angle > (SERVO_DOWN_ANGLE / 2));
    if (!wasDown && g_probeDown) g_probeDipCount++;
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
  g_manualDriveActive = false;
  motorsStop();
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  g_running = false;
  g_manualDriveActive = false;
  motorsStop();
  g_obstacleAvoiding = false;
  g_currentDriveCmd = "STOP";
  server.send(200, "text/plain", "ok");
}

void handleManualOn() {
  g_manualDriveActive = true;
  g_running = false;
  g_obstacleAvoiding = false;
  motorsStop();
  t_lastDriveCmd = millis();
  g_currentDriveCmd = "STOP";
  server.send(200, "text/plain", "ok");
}

void handleManualOff() {
  g_manualDriveActive = false;
  motorsStop();
  g_currentDriveCmd = "STOP";
  server.send(200, "text/plain", "ok");
}

void handleDrive() {
  if (!g_manualDriveActive) {
    server.send(409, "text/plain", "manual mode not active");
    return;
  }
  String cmd = server.hasArg("cmd") ? server.arg("cmd") : "STOP";
  applyDriveCommand(cmd);
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
  server.on("/api/manual_on", handleManualOn);
  server.on("/api/manual_off", handleManualOff);
  server.on("/api/drive", handleDrive);
  server.on("/api/servo", handleServoSet);
  server.on("/api/servo_auto", handleServoAuto);
  server.begin();

  t_lastStatusTick = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Aestra Ready");
  lcd.setCursor(0, 1);
  lcd.print("Waiting: START");
}

// ========================================================================
// MAIN LOOP — no delay() calls anywhere in this path, so the web server
// (and therefore every dashboard interaction) stays responsive at all
// times, even mid-maneuver or mid soil-probe-dip.
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
  }

  updateObstacleAvoidance();   // AUTO mode driving (non-blocking)
  updateDriveWatchdog();       // auto-stop MANUAL mode if joystick signal lost

  if (now - t_lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    t_lastSensorRead = now;
    readEnvironmentSensors();
  }

  updateSoilProbeCycle();      // non-blocking; overrides motors to stop while probing

  updateLcd();
}
