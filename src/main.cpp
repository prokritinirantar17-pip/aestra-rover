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
       - Premium graphite/forest dashboard: live sensor cards, selectable
         history chart, mode control (STOP / AUTO / MANUAL), soil-probe
         controls, session statistics, and an on-screen joystick for
         direct driving with a collision-safety override.

  IMPORTANT ENGINEERING NOTE — why this version is non-blocking:
    The previous version used delay() inside the soil-probe and
    obstacle-avoidance routines. Any delay() call freezes the ENTIRE
    loop(), including server.handleClient() — so while the probe was
    dipping (~2.5s), every button press / slider drag on the dashboard
    appeared to "hang". This version replaces both routines with
    millis()-based state machines that never call delay(), so the
    server stays responsive at all times, even mid-maneuver.

  DASHBOARD REVISION NOTES (this version):
    - Motor pins, sensor pins, calibration, LCD, Wi-Fi settings and the state
      machines are unchanged. Firmware changes are small and listed here:
        * /api/data gained fields: sensorsReady, dhtOk, dhtEver, probeBusy,
          obstacleCm, safetyCm, watchdogMs, histSize, histIntervalMs,
          servoMin, servoMax. Existing fields are unchanged.
        * History arrays for distance/temperature/humidity now hold "no
          reading" (JSON null) instead of fake 0 / -1 values.
        * The forward safety limit is now enforced continuously in
          updateDriveWatchdog(), not only when a drive command arrives.
        * applyDriveCommand() no longer copies arbitrary URL text into the
          telemetry JSON (unknown commands are treated as STOP).

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
#include <esp_system.h>   // esp_reset_reason(): tells us WHY the chip last rebooted

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
#define LCD_ADDRESS   0x3F
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
// ---- LCD with automatic I2C address detection ----
LiquidCrystal_I2C* lcdPtr = nullptr;
#define lcd (*lcdPtr)

void lcdBegin() {
  uint8_t found = 0;
  for (uint8_t addr = 0x20; addr <= 0x3F; addr++) {
    if (addr > 0x27 && addr < 0x38) continue;   // LCD backpacks only use 0x20-0x27 or 0x38-0x3F
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { found = addr; break; }
  }
  if (found == 0) {
    Serial.println("LCD NOT FOUND on I2C - check SDA (GPIO21), SCL (GPIO22), VCC (5V) and GND wires");
    found = LCD_ADDRESS;
  } else {
    Serial.print("LCD found at address 0x");
    Serial.println(found, HEX);
  }
  lcdPtr = new LiquidCrystal_I2C(found, LCD_COLS, LCD_ROWS);
  lcd.init();
  lcd.backlight();
}

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
bool  g_dhtOk   = false;           // most recent DHT read succeeded
bool  g_dhtEver = false;           // at least one DHT read has succeeded since boot

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

  // While the soil probe is dipping, hold the motors still. (Previously this
  // function set "forward" and the probe routine set "stop" in the same loop
  // pass, so the motors got rapid on/off pulses and current spikes.)
  if (probeState != PROBE_IDLE) {
    motorsStop();
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
    // STOP, or any unrecognised text: stop, and record a fixed value so
    // arbitrary URL text can never end up inside the telemetry JSON.
    motorsStop();
    g_currentDriveCmd = "STOP";
    return;
  }
  g_currentDriveCmd = cmd;
}

void updateDriveWatchdog() {
  if (!g_manualDriveActive) return;
  // Continuous safety: if the rover is driving forward and an obstacle
  // appears inside the safety limit, stop now instead of waiting for the
  // next joystick command to arrive.
  if (g_currentDriveCmd == "FWD" && g_distanceCm > 0 && g_distanceCm < SAFETY_STOP_CM) {
    motorsStop();
    g_safetyBlocked = true;
    g_currentDriveCmd = "STOP";
    return;
  }
  if (millis() - t_lastDriveCmd > DRIVE_WATCHDOG_MS) {
    motorsStop();
    g_currentDriveCmd = "STOP";
  }
}

// ========================================================================
// SOIL + LIGHT + DHT SENSING
// ========================================================================
void pushHistory() {
  // NAN = "no reading this cycle"; it is sent to the dashboard as JSON null.
  g_histDistance[g_histIndex] = (g_distanceCm > 0) ? g_distanceCm : NAN;
  g_histSoil[g_histIndex]     = g_soilPercent;
  g_histLight[g_histIndex]    = g_lightPercent;
  g_histTemp[g_histIndex]     = g_dhtOk ? g_temperatureC : NAN;
  g_histHumidity[g_histIndex] = g_dhtOk ? g_humidityPct  : NAN;

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
  g_dhtOk = !isnan(h) && !isnan(t);
  if (g_dhtOk) g_dhtEver = true;

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
      // Automatic dips only start while the rover is in AUTO mode. Before,
      // the servo would suddenly swing 30s after boot even when STOPPED,
      // adding an unexpected current spike on a weak power supply.
      if (g_running && !g_manualDriveActive &&
          now - t_lastSoilProbe >= SOIL_PROBE_INTERVAL_MS) {
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
    if (isnan(arr[idx])) out += "null"; else out += String(arr[idx], 1);
    if (i < count - 1) out += ",";
  }
  out += "]";
  return out;
}

// ========================================================================
// WEB DASHBOARD — premium graphite/forest theme, served from flash
// (PROGMEM), updated live via JSON polling. No external fonts/scripts,
// so it works fully offline in Access Point mode.
// ========================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="color-scheme" content="dark">
<meta name="theme-color" content="#0a0e0c">
<link rel="icon" href="data:,">
<title>AESTRA - Autonomous Agricultural Rover</title>
<style>
:root{
--bg:#0a0e0c;--s1:#131a16;--s2:#19221d;--bd:#25312a;--bd2:#34463b;
--tx:#e9f0ea;--mu:#97a89d;--dim:#6b7d72;
--gr:#5cc489;--grd:#16311f;--am:#e8ab3a;--amd:#352a0e;--rd:#ea5a52;--rdd:#3a1613;--in:#62b6cb;--ind:#10303a;
--sans:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
}
*{box-sizing:border-box}
[hidden]{display:none!important}
html{-webkit-text-size-adjust:100%}
body{margin:0;min-height:100vh;font:15px/1.45 var(--sans);color:var(--tx);font-variant-numeric:tabular-nums;
background:radial-gradient(1000px 480px at 10% -8%,#17321f66,transparent 62%),radial-gradient(800px 420px at 105% 0,#10261e55,transparent 58%),var(--bg);
padding-bottom:env(safe-area-inset-bottom)}
[data-c=gr]{--c:var(--gr)}[data-c=am]{--c:var(--am)}[data-c=rd]{--c:var(--rd)}[data-c=in]{--c:var(--in)}[data-c=off]{--c:var(--dim)}
h1,h2,p{margin:0}
h2{font-size:15px;font-weight:650;margin-bottom:12px;letter-spacing:.01em}
.note,.hint,.lbl{color:var(--mu);font-size:13px}
.note{margin-top:10px}
:focus-visible{outline:2px solid var(--gr);outline-offset:2px}

/* header */
.top{position:sticky;top:0;z-index:30;display:flex;align-items:center;gap:10px 16px;flex-wrap:wrap;padding:10px 14px;padding-top:max(10px,env(safe-area-inset-top));background:#0a0e0cd9;-webkit-backdrop-filter:blur(10px);backdrop-filter:blur(10px);border-bottom:1px solid var(--bd)}
.brand{display:flex;align-items:center;gap:10px;margin-right:auto}
.logo{width:30px;height:30px;color:var(--gr)}
.brand h1{font-size:19px;font-weight:750;letter-spacing:.14em;line-height:1.1}
.brand p{font-size:12px;color:var(--mu)}
.hstat{display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.link{display:flex;align-items:center;gap:9px}
.link b{display:block;font-size:13px;font-weight:600}
.link small{display:block;font-size:11px;color:var(--mu)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--c,var(--dim));box-shadow:0 0 0 3px #ffffff0d;flex:none}
.link[data-c=gr] .dot{animation:pulse 2.4s ease-in-out infinite}
@keyframes pulse{50%{opacity:.45}}
@keyframes spin{to{transform:rotate(360deg)}}
.pill,.tag{display:inline-flex;align-items:center;padding:4px 10px;border-radius:999px;border:1px solid var(--c,var(--bd2));color:var(--c,var(--mu));font-size:12px;font-weight:650;white-space:nowrap;background:#ffffff06}
.tag{padding:2px 8px;font-size:11px}
.banner{margin:12px auto 0;max-width:1180px;width:calc(100% - 28px);padding:11px 14px;border-radius:12px;border:1px solid var(--am);background:var(--amd);color:#f6d58f;font-size:13.5px}

/* buttons */
.btn{position:relative;font:650 14px/1 var(--sans);letter-spacing:.06em;min-height:46px;padding:0 18px;border-radius:12px;border:1px solid var(--bd2);background:var(--s2);color:var(--tx);cursor:pointer;transition:background .15s,border-color .15s,transform .08s,color .15s;touch-action:manipulation;-webkit-tap-highlight-color:transparent}
.btn:hover:not(:disabled){background:#212d26;border-color:#4a5f51}
.btn:active:not(:disabled){transform:translateY(1px) scale(.985)}
.btn:disabled{opacity:.42;cursor:not-allowed}
.btn.busy{color:transparent;pointer-events:none}
.btn.busy::after{content:"";position:absolute;inset:0;margin:auto;width:16px;height:16px;border:2px solid var(--tx);border-right-color:transparent;border-radius:50%;animation:spin .7s linear infinite}
.btn.sm{min-height:40px;font-size:13px;letter-spacing:.02em}
.btn.stop{background:var(--rdd);border-color:#7a2d28;color:#ffb8b2}
.btn.stop.on,.top .btn.stop{background:var(--rd);border-color:var(--rd);color:#1e0805}
.btn.stop:hover:not(:disabled){background:#f06d65;border-color:#f06d65;color:#1e0805}
.btn.auto.on{background:var(--gr);border-color:var(--gr);color:#06180d}
.btn.man.on{background:var(--in);border-color:var(--in);color:#04202a}
.btn.on{cursor:default}
.top .btn.stop{min-height:42px;padding:0 22px}

/* layout */
main{max-width:1180px;margin:0 auto;padding:14px 14px 28px;display:grid;gap:14px}
.row{display:grid;gap:14px}
.panel{background:linear-gradient(180deg,var(--s1),#101612);border:1px solid var(--bd);border-radius:14px;padding:16px;box-shadow:0 12px 30px -20px #000}
@media(min-width:900px){.row.two{grid-template-columns:1fr 1fr}.row.ctl{grid-template-columns:1.2fr .8fr}}
@media(max-width:700px){.top .btn.stop{order:2}.hstat{order:3;width:100%;justify-content:space-between}}

/* control centre */
.state{display:flex;align-items:center;gap:14px;padding:14px 16px;border-radius:12px;border:1px solid var(--bd2);border-left:5px solid var(--c,var(--bd2));background:var(--s2);margin-bottom:12px}
.state .dot{width:14px;height:14px}
.state[data-c=gr] .dot,.state[data-c=am] .dot,.state[data-c=in] .dot{animation:pulse 2s ease-in-out infinite}
.sname{font-size:24px;font-weight:750;line-height:1.15;color:var(--c,var(--tx))}
.ssub{font-size:13px;color:var(--mu);margin-top:2px}
.modes{display:grid;grid-template-columns:1.2fr 1fr 1fr;gap:10px}
.modes .btn{min-height:58px;font-size:15px}
.hint{margin-top:10px}

/* navigation */
.navtop{display:flex;justify-content:space-between;align-items:flex-end;gap:10px;margin-bottom:10px}
.big{font-size:40px;font-weight:700;line-height:1;letter-spacing:-.02em}
.big small{font-size:16px;color:var(--mu);font-weight:500;margin-left:4px}
.prox{--s:8%;--o:20%;position:relative;height:10px;border-radius:6px;background:linear-gradient(90deg,var(--rdd) 0 var(--s),var(--amd) var(--s) var(--o),#ffffff0d var(--o));overflow:hidden}
.prox i{position:absolute;left:0;top:0;bottom:0;width:0;border-radius:6px;background:var(--c,var(--gr));transition:width .4s ease,background .3s}
.kv{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:14px 0 0}
.kv div{background:var(--s2);border:1px solid var(--bd);border-radius:10px;padding:9px 12px}
.kv dt{font-size:12px;color:var(--mu)}.kv dd{margin:0;font-size:16px;font-weight:650}
.alert{padding:10px 12px;border-radius:10px;border:1px solid var(--c,var(--bd2));color:var(--c,var(--tx));background:#0000002e;font-size:13.5px;font-weight:600;margin-top:12px}

/* sensor cards */
.sec{margin-bottom:10px}
.cards{display:grid;gap:10px;grid-template-columns:repeat(2,1fr)}
.cards .card:last-child{grid-column:1/-1}
@media(min-width:600px){.cards{grid-template-columns:repeat(6,1fr)}.cards .card{grid-column:span 2}.cards .card:nth-child(n+4),.cards .card:last-child{grid-column:span 3}}
@media(min-width:1000px){.cards{grid-template-columns:repeat(5,1fr)}.cards .card,.cards .card:nth-child(n+4),.cards .card:last-child{grid-column:auto}}
.card{padding:14px;border-radius:12px;background:var(--s1);border:1px solid var(--bd);display:flex;flex-direction:column;gap:9px;min-width:0}
.c-h{display:flex;justify-content:space-between;align-items:center;gap:6px}
.c-t{font-size:13px;color:var(--mu);font-weight:600}
.c-v{display:flex;align-items:baseline;gap:5px}
.num{font-size:34px;font-weight:650;line-height:1;letter-spacing:-.02em}
.unit{color:var(--mu);font-size:14px}
.meter{height:6px;border-radius:4px;background:#ffffff0f;overflow:hidden}
.meter i{display:block;height:100%;width:0;background:var(--c,var(--mc));border-radius:4px;transition:width .5s ease,background .3s}
.mm{display:flex;justify-content:space-between;font-size:11px;color:var(--dim);margin-top:4px}
.c-f{display:flex;align-items:center;gap:8px;min-height:28px}
.spark{flex:1;height:28px;min-width:0}
.spark path{fill:none;stroke:var(--mc);stroke-width:1.6;vector-effect:non-scaling-stroke;stroke-linejoin:round;stroke-linecap:round}
.trend{font-size:12px;font-weight:600;color:var(--mu);white-space:nowrap}
.off .num{opacity:.45}

/* joystick */
.joy{position:relative;width:min(72vw,250px);aspect-ratio:1;margin:6px auto 10px;border-radius:50%;background:radial-gradient(circle at 50% 38%,#1d2c23,#101612 72%);border:1px solid var(--bd2);box-shadow:inset 0 0 30px #0009,0 10px 26px -14px #000;touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none;outline-offset:5px}
.joy .ring{position:absolute;inset:17%;border-radius:50%;border:1px dashed #ffffff24}
.joy .knob{position:absolute;inset:0;margin:auto;width:34%;height:34%;border-radius:50%;background:radial-gradient(circle at 35% 30%,#dcf6e6,var(--gr) 55%,#2a7549);box-shadow:0 6px 16px #000a,0 0 0 4px #5cc4891f;transition:transform .14s cubic-bezier(.2,.8,.2,1);cursor:grab;will-change:transform}
.joy.drag .knob{transition:none;cursor:grabbing}
.joy .a{position:absolute;font-size:13px;color:#ffffff3a;transition:color .12s}
.joy .a.on{color:var(--gr)}
.a.u{top:7%;left:50%;transform:translateX(-50%)}.a.d{bottom:7%;left:50%;transform:translateX(-50%)}.a.l{left:7%;top:50%;transform:translateY(-50%)}.a.r{right:7%;top:50%;transform:translateY(-50%)}
.joy.locked{opacity:.4;filter:grayscale(.7);cursor:not-allowed}
.joy.locked .knob{cursor:not-allowed}
.drv{display:flex;justify-content:space-between;gap:10px;align-items:center;margin:4px 0 12px;padding:9px 12px;border-radius:10px;background:var(--s2);border:1px solid var(--bd);font-size:13px;color:var(--mu)}
.drv b{color:var(--tx);font-size:15px}
.mpanel .btn{width:100%}

/* probe */
.probe{display:flex;align-items:center;gap:16px;margin-bottom:14px}
.dial{width:130px;flex:none}
.pang{font-size:38px;font-weight:700;line-height:1}
.pr-r{display:grid;gap:6px;justify-items:start}
.seg{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}
.seg .btn.on{background:var(--grd);border-color:var(--gr);color:var(--gr)}
input[type=range]{width:100%;height:38px;margin:4px 0 0;accent-color:var(--gr);touch-action:manipulation}
.qrow{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px}

/* history */
.tabs{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.tabs .btn{min-height:38px;padding:0 14px;font-size:13px;letter-spacing:.01em}
.tabs .btn[aria-selected=true]{background:var(--grd);border-color:var(--gr);color:var(--gr)}
.cw{position:relative;height:230px}
#chart{display:block;width:100%;height:100%;touch-action:pan-y}
.empty{position:absolute;inset:0;display:grid;place-items:center;text-align:center;padding:16px;color:var(--mu);font-size:13.5px;background:#0e1411cc;border-radius:10px}
.hs{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}
.hs div{background:var(--s2);border:1px solid var(--bd);border-radius:10px;padding:8px 10px}
.hs dt{font-size:11px;color:var(--mu)}.hs dd{margin:0;font-size:15px;font-weight:650}
@media(max-width:520px){.hs{grid-template-columns:repeat(2,1fr)}.cw{height:200px}}

/* stats */
.stats{display:grid;gap:10px;grid-template-columns:repeat(3,1fr);margin-bottom:14px}
.stat{background:var(--s2);border:1px solid var(--bd);border-radius:12px;padding:12px 14px}
.stat b{display:block;font-size:24px;font-weight:700;line-height:1.1}
.stat span{font-size:12px;color:var(--mu)}
.dist{display:flex;height:14px;border-radius:8px;overflow:hidden;background:#ffffff0d}
.dist i{height:100%;width:0;transition:width .4s}
.leg{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}
.leg div{font-size:12px;color:var(--mu)}
.leg b{display:block;color:var(--tx);font-size:14px}
.leg i{display:inline-block;width:9px;height:9px;border-radius:3px;margin-right:6px}
@media(max-width:620px){.stats{grid-template-columns:1fr 1fr}.stats .stat:first-child{grid-column:1/-1}.leg{grid-template-columns:1fr 1fr}}

footer{max-width:1180px;margin:0 auto;padding:0 14px 24px;color:var(--dim);font-size:12px}
#toasts{position:fixed;left:0;right:0;bottom:max(14px,env(safe-area-inset-bottom));display:flex;flex-direction:column;align-items:center;gap:8px;z-index:50;pointer-events:none;padding:0 14px}
.toast{max-width:460px;padding:10px 14px;border-radius:10px;font-size:13.5px;font-weight:600;background:#1b251f;border:1px solid var(--gr);color:var(--tx);box-shadow:0 10px 30px -10px #000;transition:opacity .25s}
.toast.err{border-color:var(--rd);background:#2a1513}
.toast.out{opacity:0}
@media(prefers-reduced-motion:reduce){*,*::before,*::after{animation:none!important;transition:none!important}}
</style>
</head>
<body>
<header class="top">
  <div class="brand">
    <svg class="logo" viewBox="0 0 32 32" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M6 23C6 12 12 6 26 6c0 12-6 18-16 18-1.5 0-3-.3-4-1z"/><path d="M6 27c4-8 9-12 16-15"/></svg>
    <div><h1>AESTRA</h1><p>Autonomous Agricultural Rover</p></div>
  </div>
  <div class="hstat">
    <div class="link" id="link" data-c="off" role="status" aria-live="polite"><span class="dot"></span><span><b id="linkTxt">Connecting to rover</b><small id="linkSub">Local link, no cloud sync</small></span></div>
    <span class="pill" id="sysPill" data-c="off">Waiting</span>
  </div>
  <button class="btn stop" id="hdrStop" type="button" aria-label="Stop the rover">STOP</button>
</header>
<div class="banner" id="offBanner" role="alert" hidden></div>

<main id="app">
<div class="row ctl">
  <section class="panel" aria-labelledby="hCtl">
    <h2 id="hCtl">Rover control centre</h2>
    <div class="state" id="stateBox" data-c="off">
      <span class="dot"></span>
      <div><div class="sname" id="stateName">Waiting for rover</div><div class="ssub" id="stateSub">Connecting to the rover over the local link.</div></div>
    </div>
    <div class="modes" role="group" aria-label="Operating mode">
      <button class="btn stop" id="bStop" type="button" aria-label="Stop the rover">STOP</button>
      <button class="btn auto" id="bAuto" type="button" aria-pressed="false">AUTO</button>
      <button class="btn man" id="bMan" type="button" aria-pressed="false">MANUAL</button>
    </div>
    <p class="hint">STOP halts the motors at once and leaves both modes. The Esc key also sends STOP.</p>
  </section>

  <section class="panel" aria-labelledby="hNav">
    <h2 id="hNav">Autonomous navigation</h2>
    <div class="navtop"><div><div class="big"><span id="navDist">--</span><small id="navUnit" hidden>cm</small></div><div class="lbl">Distance to nearest obstacle</div></div><span class="pill" id="navPill" data-c="off">--</span></div>
    <div class="prox" id="prox" role="img" aria-label="Obstacle distance on a 0 to 100 centimetre scale"><i id="proxFill"></i></div>
    <div class="mm"><span>0 cm</span><span id="proxLeg">Distance scale</span><span>100+ cm</span></div>
    <div class="alert" id="navAlert" hidden></div>
    <dl class="kv"><div><dt>Navigation</dt><dd id="navMode">--</dd></div><div><dt>Obstacles avoided</dt><dd id="navCount">--</dd></div></dl>
  </section>
</div>

<section aria-labelledby="hSen">
  <h2 class="sec" id="hSen">Live sensor telemetry</h2>
  <div class="cards" id="cards"></div>
</section>

<div class="row two">
  <section class="panel mpanel" aria-labelledby="hMan">
    <h2 id="hMan">Manual driving</h2>
    <div class="joy locked" id="joy" tabindex="0" role="application" aria-disabled="true" aria-label="Drive joystick. Arrow keys drive, release to stop.">
      <div class="ring"></div>
      <span class="a u" id="aFWD">&#9650;</span><span class="a d" id="aBACK">&#9660;</span><span class="a l" id="aLEFT">&#9664;</span><span class="a r" id="aRIGHT">&#9654;</span>
      <div class="knob" id="knob"></div>
    </div>
    <div class="drv"><span>Rover is doing</span><b id="cmdNow">--</b></div>
    <div class="alert" id="safeAlert" data-c="rd" hidden>Forward blocked: obstacle closer than the safety limit.</div>
    <button class="btn man" id="manToggle" type="button">Enable manual mode</button>
    <p class="note" id="manHint">Switch to manual mode to unlock the joystick.</p>
  </section>

  <section class="panel" aria-labelledby="hProbe">
    <h2 id="hProbe">Soil probe</h2>
    <div class="probe">
      <svg class="dial" viewBox="0 0 120 90" aria-hidden="true"><path d="M20 18A58 58 0 0 1 78 76" fill="none" stroke="#34463b" stroke-width="2" stroke-dasharray="3 4"/><line id="arm" x1="20" y1="76" x2="20" y2="18" stroke="#5cc489" stroke-width="5" stroke-linecap="round"/><circle cx="20" cy="76" r="6" fill="#19221d" stroke="#5cc489" stroke-width="2"/></svg>
      <div class="pr-r"><div class="pang" id="pAng">--</div><div class="lbl">Commanded angle</div><span class="pill" id="pState" data-c="off">--</span></div>
    </div>
    <div class="seg" role="group" aria-label="Probe control mode">
      <button class="btn sm" id="pAuto" type="button" aria-pressed="false">Automatic cycle</button>
      <button class="btn sm" id="pMan" type="button" aria-pressed="false">Manual control</button>
    </div>
    <label class="lbl" for="servoSlider">Probe angle</label>
    <input id="servoSlider" type="range" min="0" max="90" step="1" value="0" disabled>
    <div class="mm"><span>0&deg; raised</span><span>90&deg; lowered</span></div>
    <div class="qrow"><button class="btn sm" id="pRaise" type="button">Raise probe</button><button class="btn sm" id="pLower" type="button">Lower probe</button></div>
    <div class="alert" id="pWarn" data-c="am" hidden>The probe is commanded down while the rover can drive. Raise it first.</div>
    <p class="note" id="pNote">Angle is the commanded position. The servo has no position feedback.</p>
  </section>
</div>

<section class="panel" aria-labelledby="hHist">
  <h2 id="hHist">Telemetry history</h2>
  <div class="tabs" id="tabs" role="tablist" aria-label="Metric to chart"></div>
  <div class="cw"><canvas id="chart" role="img" aria-label="History chart of the selected metric"></canvas><div class="empty" id="chartEmpty">Collecting samples</div></div>
  <dl class="hs"><div><dt>Latest</dt><dd id="hLast">--</dd></div><div><dt>Lowest</dt><dd id="hMin">--</dd></div><div><dt>Highest</dt><dd id="hMax">--</dd></div><div><dt>Valid samples</dt><dd id="hN">--</dd></div></dl>
  <p class="note" id="histNote">History is kept in the rover's memory and clears on reboot.</p>
</section>

<section class="panel" aria-labelledby="hStat">
  <h2 id="hStat">Session statistics</h2>
  <div class="stats">
    <div class="stat"><b id="stUp">--</b><span>Uptime since boot</span></div>
    <div class="stat"><b id="stAvoid">--</b><span>Obstacles avoided</span></div>
    <div class="stat"><b id="stDips">--</b><span>Soil-probe dips</span></div>
  </div>
  <div class="dist" id="dist" role="img" aria-label="Share of time by mode"></div>
  <div class="leg" id="leg"></div>
  <p class="note">Shares are measured from the rover's own timers since boot. Navigating includes time holding still for the probe.</p>
</section>
</main>

<footer>The rover serves this dashboard itself. Everything is embedded, so no internet or cloud connection is used or needed.</footer>
<div id="toasts" role="status" aria-live="polite"></div>

<script>
(function(){
'use strict';
var $=function(id){return document.getElementById(id);};
var sleep=function(ms){return new Promise(function(r){setTimeout(r,ms);});};
var setT=function(el,t){if(el.textContent!==t)el.textContent=t;};
var setC=function(el,c){if(c){if(el.dataset.c!==c)el.dataset.c=c;}else if(el.hasAttribute('data-c'))el.removeAttribute('data-c');};

var M=[
{k:'dist',f:'distance',hk:'historyDistance',n:'Obstacle distance',u:'cm',d:0,c:'#7cc4b0',lo:'0',hi:'200 cm',min:0,max:200},
{k:'soil',f:'soil',hk:'historySoil',n:'Soil moisture',u:'%',d:0,c:'#6fbf73',lo:'Dry',hi:'Wet',min:0,max:100},
{k:'light',f:'light',hk:'historyLight',n:'Ambient light',u:'%',d:0,c:'#e0b84f',lo:'Dark',hi:'Bright',min:0,max:100},
{k:'temp',f:'temp',hk:'historyTemp',n:'Temperature',u:'\u00b0C',d:1,c:'#e0825a',lo:'0',hi:'50 \u00b0C',min:0,max:50},
{k:'hum',f:'humidity',hk:'historyHumidity',n:'Humidity',u:'%',d:0,c:'#6fa8d8',lo:'0',hi:'100 %',min:0,max:100}];
var MM={};M.forEach(function(m){MM[m.k]=m;});
var SEG=[['statusNavigating','Navigating','#5cc489'],['statusAvoiding','Avoiding','#e8ab3a'],['statusManual','Manual','#62b6cb'],['statusStopped','Stopped','#ea5a52']];
var ST={
OFFLINE:{n:'Disconnected',c:'off'},STOPPED:{n:'Stopped',c:'rd',s:'Motors are off. Choose AUTO or MANUAL.'},
NAVIGATING:{n:'Navigating',c:'gr',s:'AUTO mode: driving forward with the path clear.'},
AVOIDING:{n:'Avoiding obstacle',c:'am',s:'AUTO mode: obstacle detected, turning away.'},
SAMPLING:{n:'Sampling soil',c:'gr',s:'AUTO mode: holding still while the probe cycle runs.'},
MANUAL:{n:'Manual driving',c:'in',s:'MANUAL mode: you control the drive with the joystick.'}};
var CMDW={FWD:'Driving forward',BACK:'Reversing',LEFT:'Turning left',RIGHT:'Turning right',STOP:'Stopped'};

var S={d:null,ok:false,ever:false,wasOk:false,fails:0,lastOk:0,inPoll:false,again:false,busy:{mode:0,servo:0},stopN:0,metric:'soil',sig:'',hover:-1,sk:{}};
var g=function(k,def){return S.d&&S.d[k]!=null?S.d[k]:def;};

/* ---------- networking ---------- */
async function api(path,ms,sig,json){
  var ac=new AbortController(),t=setTimeout(function(){ac.abort();},ms);
  if(sig){if(sig.aborted)ac.abort();else sig.addEventListener('abort',function(){ac.abort();},{once:true});}
  try{
    var r=await fetch(path,{cache:'no-store',signal:ac.signal});
    var data=json&&r.ok?await r.json():null;
    return {ok:r.ok,status:r.status,data:data};
  }catch(e){return {ok:false,status:0,data:null};}
  finally{clearTimeout(t);}
}
function toast(msg,kind){
  var box=$('toasts'),el=document.createElement('div');
  el.className='toast'+(kind==='err'?' err':'');el.setAttribute('role',kind==='err'?'alert':'status');el.textContent=msg;box.appendChild(el);
  while(box.children.length>3)box.removeChild(box.firstChild);
  setTimeout(function(){el.classList.add('out');setTimeout(function(){el.remove();},260);},kind==='err'?6500:2800);
}
async function poll(){
  if(S.inPoll){S.again=true;return;}
  S.inPoll=true;
  var r=await api('/api/data',2500,null,true);
  S.inPoll=false;
  if(r.ok&&r.data){S.fails=0;S.lastOk=Date.now();S.ever=true;S.d=r.data;link();render();}
  else{S.fails++;link();}
  if(S.again){S.again=false;poll();}
}

/* ---------- connection state ---------- */
function stateKey(){
  var d=S.d;
  if(!S.ok)return 'OFFLINE';
  if(d.manualDrive)return 'MANUAL';
  if(!d.running)return 'STOPPED';
  if(d.avoiding)return 'AVOIDING';
  if(d.probeBusy)return 'SAMPLING';
  return 'NAVIGATING';
}
function link(){
  var age=Date.now()-S.lastOk,was=S.ok;
  S.ok=S.lastOk>0&&age<3500;
  var el=$('link');
  if(!S.ever&&S.fails>=2){setC(el,'rd');setT($('linkTxt'),'Rover not reachable');setT($('linkSub'),'Check you are on the rover Wi-Fi. Retrying.');}
  else if(!S.ever){setC(el,'off');setT($('linkTxt'),'Connecting to rover');setT($('linkSub'),'Local link, no cloud sync');}
  else if(!S.ok){setC(el,'rd');setT($('linkTxt'),'Rover disconnected');setT($('linkSub'),'Retrying. Last response '+new Date(S.lastOk).toLocaleTimeString());}
  else if(S.fails>0){setC(el,'am');setT($('linkTxt'),'Link unstable');setT($('linkSub'),'Last response '+new Date(S.lastOk).toLocaleTimeString());}
  else{setC(el,'gr');setT($('linkTxt'),'Rover connected');setT($('linkSub'),'Local link, no cloud sync. Updated '+new Date(S.lastOk).toLocaleTimeString());}
  document.body.classList.toggle('off',S.ever&&!S.ok);
  if(was&&!S.ok){driveCancel();toast('Lost contact with the rover','err');}
  if(!was&&S.ok&&S.wasOk)toast('Reconnected to the rover');
  if(S.ok)S.wasOk=true;
  if(was!==S.ok&&S.d)render();
  applyEnable();
}

/* ---------- rendering ---------- */
function render(){
  var d=S.d;if(!d)return;
  var k=stateKey(),s=ST[k],sub=s.s;
  if(k==='OFFLINE'){
    sub='Last known mode: '+(d.manualDrive?'MANUAL':d.running?'AUTO':'stopped')+'. '+(d.running?'The rover keeps running AUTO on its own. Press STOP as soon as the link returns.':d.manualDrive?'Manual driving halts by itself within '+(g('watchdogMs',800)/1000).toFixed(1)+' s without commands.':'The motors were off.');
  }
  setC($('stateBox'),s.c);setT($('stateName'),s.n);setT($('stateSub'),sub);
  setC($('sysPill'),s.c);setT($('sysPill'),s.n);
  var off=$('offBanner');
  if(k==='OFFLINE'){off.hidden=false;setT(off,'Dashboard link lost. Readings are the last values received and may be out of date.');}else off.hidden=true;
  var mode=d.manualDrive?'MANUAL':d.running?'AUTO':'STOP';
  [['bStop','STOP'],['bAuto','AUTO'],['bMan','MANUAL']].forEach(function(p){
    var b=$(p[0]),on=mode===p[1];b.classList.toggle('on',on);if(p[0]!=='bStop')b.setAttribute('aria-pressed',on?'true':'false');
  });
  nav(d,k);M.forEach(function(m){card(m,d);});man(d);probe(d);chart();stats(d);
  setT($('histNote'),'Each point is one rover sensor cycle, about '+(g('histIntervalMs',2000)/1000)+' s apart. History is held in rover memory and clears on reboot.');
  applyEnable();
}
function nav(d,k){
  var dist=d.distance,so=g('safetyCm',8),ob=g('obstacleCm',20),has=dist>0;
  setT($('navDist'),has?dist.toFixed(0):'--');
  $('navUnit').hidden=!has;
  var p=$('navPill'),pc='off',pt='No echo';
  if(has){if(dist<so){pc='rd';pt='Blocked';}else if(dist<ob){pc='am';pt='Obstacle';}else{pc='gr';pt='Clear';}}
  setC(p,pc);setT(p,pt);
  var pr=$('prox');pr.style.setProperty('--s',so+'%');pr.style.setProperty('--o',ob+'%');
  var f=$('proxFill');f.style.width=has?Math.min(100,dist)+'%':'0%';setC(f,has?pc:null);
  setT($('proxLeg'),'Red under '+so+' cm, amber under '+ob+' cm');
  var a=$('navAlert'),txt='',ac='';
  if(d.running&&d.avoiding){txt='Obstacle detected. Stopping, turning and resuming.';ac='am';}
  else if(d.manualDrive&&d.safetyBlocked){txt='Forward drive blocked: obstacle closer than '+so+' cm.';ac='rd';}
  else if(has&&dist<ob&&dist>=so){txt='Obstacle within '+ob+' cm.';ac='am';}
  else if(has&&dist<so){txt='Obstacle within '+so+' cm.';ac='rd';}
  else if(d.running&&d.probeBusy){txt='Holding position for the soil-probe cycle.';ac='gr';}
  a.hidden=!txt;if(txt){setT(a,txt);setC(a,ac);}
  setT($('navMode'),d.running?(k==='OFFLINE'?'AUTO (last known)':ST[k].n):d.manualDrive?'Off (manual mode)':'Off (stopped)');
  setT($('navCount'),String(d.obstaclesAvoided));
}
function spark(m,h){
  var el=$('s_'+m.k),pts=h.filter(function(v){return v!==null;});
  if(h.length<2||pts.length<2){el.setAttribute('d','');return;}
  var lo=Math.min.apply(null,pts),hi=Math.max.apply(null,pts);if(hi-lo<1e-9){lo-=1;hi+=1;}
  var s='',pen=false;
  h.forEach(function(v,i){if(v===null){pen=false;return;}
    s+=(pen?'L':'M')+((i/(h.length-1))*100).toFixed(1)+' '+(26-((v-lo)/(hi-lo))*24).toFixed(1);pen=true;});
  if(S.sk[m.k]!==s){S.sk[m.k]=s;el.setAttribute('d',s);}
}
function trend(m,h){
  var el=$('r_'+m.k);
  if(m.k==='dist'||h.length<6){setT(el,'');return;}
  var n=Math.min(10,h.length-1),a=h[h.length-1],b=h[h.length-1-n];
  if(a===null||b===null){setT(el,'');return;}
  var df=a-b;el.title='Change over the last '+n+' samples';
  setT(el,Math.abs(df)<(m.d?0.05:0.5)?'\u25AC steady':(df>0?'\u25B2 +':'\u25BC \u2212')+Math.abs(df).toFixed(m.d));
}
function card(m,d){
  var v=null,tag='No data',tc='off',gc=null;
  if(m.k==='dist'){
    if(d.distance>0){v=d.distance;var so=g('safetyCm',8),ob=g('obstacleCm',20);
      if(v<so){tag='Blocked';tc='rd';}else if(v<ob){tag='Caution';tc='am';}else{tag='Clear';tc='gr';}gc=tc;}
    else tag='No echo';
  }else if(m.k==='soil'||m.k==='light'){if(d.sensorsReady){v=d[m.f];tag='Live';tc='gr';}}
  else if(d.dhtEver){v=d[m.f];if(d.dhtOk){tag='Live';tc='gr';}else{tag='Stale';tc='am';}}
  if(!S.ok&&v!==null){tag='Last value';tc='off';}
  setC($('t_'+m.k),tc);setT($('t_'+m.k),tag);
  setT($('v_'+m.k),v===null?'--':v.toFixed(m.d));
  var gEl=$('g_'+m.k);gEl.style.width=v===null?'0%':Math.max(0,Math.min(100,(v-m.min)/(m.max-m.min)*100)).toFixed(1)+'%';setC(gEl,gc);
  var h=d[m.hk]||[];spark(m,h);trend(m,h);
}
function man(d){
  var on=!!d.manualDrive;
  setT($('cmdNow'),on?(CMDW[d.driveCmd]||'Stopped'):'Manual mode off');
  $('safeAlert').hidden=!(on&&d.safetyBlocked);
  setT($('safeAlert'),'Forward blocked: obstacle closer than '+g('safetyCm',8)+' cm. Reverse or turn to continue.');
  setT($('manToggle'),on?'Exit manual mode':'Enable manual mode');
  setT($('manHint'),on?'Drag or touch the stick to drive. Let go to stop. With the stick focused, arrow keys or W A S D also drive.':S.ok?'Switch to manual mode to unlock the joystick.':'Disconnected: driving is unavailable.');
  if(!on&&(J.active||J.sent!=='STOP'))driveCancel(true);
}
function probe(d){
  var mn=g('servoMin',0),mx=g('servoMax',90),sl=$('servoSlider');
  if(sl.min!==String(mn))sl.min=mn;if(sl.max!==String(mx))sl.max=mx;
  var ang=SV.edit?+sl.value:d.servoAngle;
  if(!SV.edit&&!SV.busy&&SV.pend===null)sl.value=d.servoAngle;
  setT($('pAng'),ang+'\u00b0');
  var rad=(ang-mn)/(mx-mn||1)*Math.PI/2,arm=$('arm');
  arm.setAttribute('x2',(20+58*Math.sin(rad)).toFixed(1));arm.setAttribute('y2',(76-58*Math.cos(rad)).toFixed(1));
  var ps=d.probeBusy?'Auto cycle running':d.probeDown?'Lowered':d.servoAngle<=mn?'Raised':'Partly lowered';
  setC($('pState'),d.probeDown?'am':'gr');setT($('pState'),ps);
  [['pAuto',!d.manualServo],['pMan',!!d.manualServo]].forEach(function(p){var b=$(p[0]);b.classList.toggle('on',p[1]);b.setAttribute('aria-pressed',p[1]?'true':'false');});
  $('pWarn').hidden=!((d.running||d.manualDrive)&&d.manualServo&&d.probeDown);
  setT($('pNote'),d.manualServo?'Manual control: the automatic dip cycle is paused. Angle is the commanded position; the servo has no position feedback.':'Automatic cycle: the probe dips on its own timer while AUTO is running.');
}

/* ---------- history chart ---------- */
function geo(w,h){return {L:44,R:12,T:12,B:26,w:w,h:h};}
function chart(){
  var d=S.d,cv=$('chart');if(!d||!cv.clientWidth)return;
  var m=MM[S.metric],h=d[m.hk]||[],w=cv.clientWidth,H=cv.clientHeight,dpr=window.devicePixelRatio||1;
  var sig=S.metric+'|'+w+'x'+H+'|'+dpr+'|'+S.hover+'|'+h.join(',');
  if(sig===S.sig)return;S.sig=sig;
  var vals=h.filter(function(v){return v!==null;}),N=g('histSize',30);
  var empty=$('chartEmpty');
  setT($('hN'),vals.length+' of '+h.length);
  if(vals.length){var lo0=Math.min.apply(null,vals),hi0=Math.max.apply(null,vals),u=m.u==='%'?' %':' '+m.u;
    setT($('hLast'),h[h.length-1]===null?'--':h[h.length-1].toFixed(m.d)+u);setT($('hMin'),lo0.toFixed(m.d)+u);setT($('hMax'),hi0.toFixed(m.d)+u);}
  else{setT($('hLast'),'--');setT($('hMin'),'--');setT($('hMax'),'--');}
  var x=cv.getContext('2d');
  if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(H*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(H*dpr);}
  x.setTransform(dpr,0,0,dpr,0,0);x.clearRect(0,0,w,H);
  if(vals.length<2){empty.hidden=false;setT(empty,vals.length?'Collecting samples. One more reading is needed to draw a trend.':h.length?'No valid '+m.n.toLowerCase()+' readings yet.':'Collecting samples.');return;}
  empty.hidden=true;
  var lo,hi;
  if(m.k==='temp'){lo=Math.floor(Math.min.apply(null,vals)-2);hi=Math.ceil(Math.max.apply(null,vals)+2);if(hi-lo<6){var c=(hi+lo)/2;lo=Math.floor(c-3);hi=Math.ceil(c+3);}}
  else if(m.k==='dist'){lo=0;hi=Math.max(100,Math.ceil(Math.max.apply(null,vals)*1.1/50)*50);}
  else{lo=0;hi=100;}
  var G=geo(w,H),pw=w-G.L-G.R,ph=H-G.T-G.B,step=pw/(N-1);
  var X=function(i){return G.L+pw-(h.length-1-i)*step;},Y=function(v){return G.T+ph-(v-lo)/(hi-lo)*ph;};
  x.font='11px '+getComputedStyle(document.body).fontFamily;x.textBaseline='middle';
  for(var t=0;t<=4;t++){
    var tv=lo+(hi-lo)*t/4,ty=Y(tv);
    x.strokeStyle='#25312a';x.lineWidth=1;x.beginPath();x.moveTo(G.L,ty+.5);x.lineTo(w-G.R,ty+.5);x.stroke();
    x.fillStyle='#97a89d';x.textAlign='right';x.fillText((Math.round(tv*10)/10).toString(),G.L-6,ty);
  }
  x.textBaseline='alphabetic';x.fillStyle='#6b7d72';
  x.textAlign='left';x.fillText('-'+(N-1),G.L,H-6);x.textAlign='center';x.fillText('samples ago',G.L+pw/2,H-6);x.textAlign='right';x.fillText('now',w-G.R,H-6);
  var seg=[],cur=[];
  h.forEach(function(v,i){if(v===null){if(cur.length)seg.push(cur);cur=[];}else cur.push([X(i),Y(v)]);});
  if(cur.length)seg.push(cur);
  seg.forEach(function(s){
    if(s.length===1){x.fillStyle=m.c;x.beginPath();x.arc(s[0][0],s[0][1],2.5,0,7);x.fill();return;}
    x.beginPath();s.forEach(function(p,i){i?x.lineTo(p[0],p[1]):x.moveTo(p[0],p[1]);});
    x.lineTo(s[s.length-1][0],G.T+ph);x.lineTo(s[0][0],G.T+ph);x.closePath();x.fillStyle=m.c+'22';x.fill();
    x.beginPath();s.forEach(function(p,i){i?x.lineTo(p[0],p[1]):x.moveTo(p[0],p[1]);});
    x.strokeStyle=m.c;x.lineWidth=2;x.lineJoin='round';x.stroke();
  });
  var hv=S.hover>=0&&S.hover<h.length&&h[S.hover]!==null?S.hover:h.length-1;
  while(hv>=0&&h[hv]===null)hv--;
  if(hv>=0){
    var px=X(hv),py=Y(h[hv]);
    if(S.hover>=0){x.strokeStyle='#ffffff40';x.beginPath();x.moveTo(px+.5,G.T);x.lineTo(px+.5,G.T+ph);x.stroke();}
    x.fillStyle='#0a0e0c';x.strokeStyle=m.c;x.lineWidth=2;x.beginPath();x.arc(px,py,4,0,7);x.fill();x.stroke();
    var label=h[hv].toFixed(m.d)+(m.u==='%'?' %':' '+m.u)+(hv===h.length-1?' (latest)':' ('+(h.length-1-hv)+' samples ago)');
    x.font='600 12px '+getComputedStyle(document.body).fontFamily;
    var tw=x.measureText(label).width+14,bx=Math.min(Math.max(px-tw/2,G.L),w-G.R-tw),by=py>G.T+34?py-30:py+10;
    x.fillStyle='#19221d';x.strokeStyle='#34463b';x.lineWidth=1;x.fillRect(bx,by,tw,22);x.strokeRect(bx+.5,by+.5,tw,22);
    x.fillStyle='#e9f0ea';x.textAlign='left';x.textBaseline='middle';x.fillText(label,bx+7,by+11);
  }
}
function stats(d){
  setT($('stUp'),fmtDur(d.uptimeMs,true));setT($('stAvoid'),String(d.obstaclesAvoided));setT($('stDips'),String(d.probeDips));
  var tot=0;SEG.forEach(function(s){tot+=d[s[0]]||0;});
  SEG.forEach(function(s,i){
    var ms=d[s[0]]||0,pc=tot?ms/tot*100:0;
    $('sg'+i).style.width=pc.toFixed(2)+'%';
    setT($('sv'+i),fmtDur(ms)+' ('+pc.toFixed(0)+'%)');
  });
  $('dist').setAttribute('aria-label','Share of time by mode: '+SEG.map(function(s){return s[1]+' '+(tot?((d[s[0]]||0)/tot*100).toFixed(0):0)+'%';}).join(', '));
}
function fmtDur(ms,withH){
  var s=Math.floor((ms||0)/1000),h=Math.floor(s/3600),m=Math.floor(s%3600/60),r=s%60,p=function(n){return n<10?'0'+n:''+n;};
  return (h||withH?h+':':'')+p(m)+':'+p(r);
}

/* ---------- control enable/disable ---------- */
function applyEnable(){
  var ok=S.ok,d=S.d,man=!!(d&&d.manualDrive),sm=!!(d&&d.manualServo),mb=S.busy.mode,sb=S.busy.servo;
  $('bAuto').disabled=!ok||mb;$('bMan').disabled=!ok||mb;$('manToggle').disabled=!ok||mb;
  $('pAuto').disabled=!ok||sb;$('pMan').disabled=!ok||sb;
  $('servoSlider').disabled=!ok||!sm;$('pRaise').disabled=!ok||!sm;$('pLower').disabled=!ok||!sm;
  var can=driveOK();
  $('joy').classList.toggle('locked',!can);$('joy').setAttribute('aria-disabled',can?'false':'true');
}

/* ---------- mode + STOP ---------- */
async function modeCmd(path,okMsg,btn){
  if(S.busy.mode)return;
  S.busy.mode=1;btn.classList.add('busy');applyEnable();
  driveCancel(true);
  var r=await api(path,3000);
  S.busy.mode=0;btn.classList.remove('busy');
  if(r.ok)toast(okMsg);else toast('Mode change failed: no confirmation from the rover.','err');
  await poll();applyEnable();
}
var lastStop=0;
async function doStop(){
  var now=Date.now();if(S.stopN>0&&now-lastStop<400)return;
  lastStop=now;S.stopN++;driveCancel(true);
  ['bStop','hdrStop'].forEach(function(i){$(i).setAttribute('aria-busy','true');});
  var ok=false;
  for(var i=0;i<3&&!ok;i++){var r=await api('/api/stop',1500);ok=r.ok;if(!ok)await sleep(300);}
  S.stopN--;
  if(S.stopN===0)['bStop','hdrStop'].forEach(function(i){$(i).removeAttribute('aria-busy');});
  if(ok)toast('Rover stopped');else toast('STOP not confirmed. The rover may still be moving. Press STOP again or cut power.','err');
  poll();
}
$('bStop').addEventListener('click',doStop);$('hdrStop').addEventListener('click',doStop);
$('bAuto').addEventListener('click',function(){if(S.d&&!S.d.manualDrive&&S.d.running)return;modeCmd('/api/start','AUTO mode on',$('bAuto'));});
$('bMan').addEventListener('click',function(){if(S.d&&S.d.manualDrive)return;modeCmd('/api/manual_on','Manual mode on',$('bMan'));});
$('manToggle').addEventListener('click',function(){
  if(S.d&&S.d.manualDrive)modeCmd('/api/manual_off','Manual mode off, rover stopped',$('manToggle'));
  else modeCmd('/api/manual_on','Manual mode on',$('manToggle'));
});
document.addEventListener('keydown',function(e){if(e.key==='Escape'){doStop();}});

/* ---------- joystick ---------- */
var J={zone:$('joy'),knob:$('knob'),active:false,id:null,cmd:'STOP',sent:'STOP',rect:null,busy:false,fail:0,t:0,ac:null,keys:[],stopId:0};
function driveOK(){return !!(S.ok&&S.d&&S.d.manualDrive&&S.stopN===0&&!S.busy.mode);}
function jmax(){return J.zone.clientWidth/2-J.knob.offsetWidth/2-4;}
function knobTo(x,y){J.knob.style.transform='translate('+x.toFixed(1)+'px,'+y.toFixed(1)+'px)';}
function setDir(cmd){['FWD','BACK','LEFT','RIGHT'].forEach(function(c){$('a'+c).classList.toggle('on',c===cmd);});}
function setCmd(cmd){if(cmd!==J.cmd){J.cmd=cmd;setDir(cmd);driveKick();}}
function driveKick(){clearTimeout(J.t);driveTick();}
async function driveTick(){
  if(J.busy||!J.active)return;
  if(!driveOK()){driveCancel(true);return;}
  J.busy=true;var cmd=J.cmd;J.sent=cmd;J.ac=new AbortController();
  var r=await api('/api/drive?cmd='+cmd,1000,J.ac.signal);
  J.busy=false;J.ac=null;
  if(!J.active)return;
  if(r.ok)J.fail=0;
  else if(r.status===409){driveCancel(true);toast('Manual mode is not active on the rover.','err');poll();return;}
  else if(++J.fail>=2){driveCancel();toast('Drive link lost. The rover stops itself within a second.','err');return;}
  var gap=Math.min(200,g('watchdogMs',800)/4);
  J.t=setTimeout(driveTick,cmd!==J.cmd?0:gap);
}
function driveCancel(skipSend){
  var need=J.active||J.sent!=='STOP';
  J.active=false;J.id=null;J.keys.length=0;J.cmd='STOP';J.fail=0;clearTimeout(J.t);
  J.zone.classList.remove('drag');knobTo(0,0);setDir('STOP');
  if(J.ac)J.ac.abort();
  if(need&&!skipSend)sendStop();else if(skipSend)J.sent='STOP';
}
async function sendStop(){
  var id=++J.stopId;
  for(var i=0;i<3;i++){
    var r=await api('/api/drive?cmd=STOP',1200);
    if(id!==J.stopId)return;
    if(r.ok){J.sent='STOP';setTimeout(function(){if(!J.active&&S.d&&S.d.manualDrive)api('/api/drive?cmd=STOP',1200);},200);return;}
    if(r.status===409){J.sent='STOP';return;}
    await sleep(250);
  }
  toast('Could not confirm the stop. The rover stops itself within a second.','err');
}
function joyMove(cx,cy){
  var r=J.rect,R=r.width/2,mx=jmax(),dx=cx-(r.left+R),dy=cy-(r.top+r.height/2);
  var dd=Math.hypot(dx,dy),cl=Math.min(dd,mx),a=Math.atan2(dy,dx);
  knobTo(Math.cos(a)*cl,Math.sin(a)*cl);
  var cmd='STOP';
  if(cl>mx*0.3)cmd=Math.abs(dx)>Math.abs(dy)?(dx>0?'RIGHT':'LEFT'):(dy>0?'BACK':'FWD');
  setCmd(cmd);
}
J.zone.addEventListener('pointerdown',function(e){
  if(!driveOK()||J.active||(e.pointerType==='mouse'&&e.button!==0))return;
  e.preventDefault();J.active=true;J.id=e.pointerId;J.stopId++;
  try{J.zone.setPointerCapture(e.pointerId);}catch(_){}
  J.rect=J.zone.getBoundingClientRect();J.zone.classList.add('drag');
  joyMove(e.clientX,e.clientY);driveKick();
});
J.zone.addEventListener('pointermove',function(e){if(J.active&&e.pointerId===J.id)joyMove(e.clientX,e.clientY);});
['pointerup','pointercancel','lostpointercapture'].forEach(function(t){J.zone.addEventListener(t,function(e){if(J.active&&e.pointerId===J.id)driveCancel();});});
J.zone.addEventListener('contextmenu',function(e){e.preventDefault();});
window.addEventListener('blur',function(){driveCancel();});
window.addEventListener('pagehide',function(){driveCancel();});
document.addEventListener('visibilitychange',function(){if(document.hidden)driveCancel();});
var KM={ArrowUp:'FWD',ArrowDown:'BACK',ArrowLeft:'LEFT',ArrowRight:'RIGHT',w:'FWD',s:'BACK',a:'LEFT',d:'RIGHT'};
var KV={FWD:[0,-1],BACK:[0,1],LEFT:[-1,0],RIGHT:[1,0]};
function kbApply(){
  if(!J.keys.length){driveCancel();return;}
  var c=J.keys[J.keys.length-1],v=KV[c],mx=jmax();knobTo(v[0]*mx*0.8,v[1]*mx*0.8);setCmd(c);
}
J.zone.addEventListener('keydown',function(e){
  var c=KM[e.key.length===1?e.key.toLowerCase():e.key];if(!c||!driveOK())return;
  e.preventDefault();if(e.repeat)return;
  if(!J.active){J.active=true;J.id=null;J.stopId++;J.zone.classList.add('drag');}
  J.keys=J.keys.filter(function(k){return k!==c;});J.keys.push(c);kbApply();
});
J.zone.addEventListener('keyup',function(e){
  var c=KM[e.key.length===1?e.key.toLowerCase():e.key];if(!c||J.id!==null)return;
  J.keys=J.keys.filter(function(k){return k!==c;});if(J.active)kbApply();
});
J.zone.addEventListener('blur',function(){driveCancel();});

/* ---------- soil probe ---------- */
var SV={busy:false,pend:null,edit:false,t:0,msg:null};
async function servoFlush(){
  SV.busy=true;S.busy.servo=1;applyEnable();
  var ok=true,last=null;
  while(SV.pend!==null){
    last=SV.pend;SV.pend=null;
    var r=await api('/api/servo?angle='+last,2000);
    if(!r.ok){ok=false;SV.pend=null;break;}
    await sleep(70);
  }
  SV.busy=false;S.busy.servo=0;
  if(ok&&SV.msg){toast(SV.msg.replace('{a}',last));}
  else if(!ok)toast('Probe command failed. Position not confirmed.','err');
  SV.msg=null;poll();applyEnable();
}
function servoSend(a,msg){SV.pend=a;if(msg)SV.msg=msg;if(!SV.busy)servoFlush();}
var sl=$('servoSlider');
sl.addEventListener('input',function(){
  SV.edit=true;clearTimeout(SV.t);SV.t=setTimeout(function(){SV.edit=false;},900);
  setT($('pAng'),sl.value+'\u00b0');servoSend(+sl.value);
});
sl.addEventListener('change',function(){servoSend(+sl.value,'Probe command accepted: {a}\u00b0');});
$('pRaise').addEventListener('click',function(){var v=+sl.min;sl.value=v;servoSend(v,'Raise command accepted ({a}\u00b0)');});
$('pLower').addEventListener('click',function(){var v=+sl.max;sl.value=v;servoSend(v,'Lower command accepted ({a}\u00b0)');});
$('pMan').addEventListener('click',function(){if(S.d&&!S.d.manualServo)servoSend(S.d.servoAngle,'Manual probe control on');});
$('pAuto').addEventListener('click',async function(){
  if(S.busy.servo||(S.d&&!S.d.manualServo))return;S.busy.servo=1;var b=$('pAuto');b.classList.add('busy');applyEnable();
  var r=await api('/api/servo_auto',2500);
  S.busy.servo=0;b.classList.remove('busy');
  if(r.ok)toast('Automatic probe cycle on. Probe raised.');else toast('Could not switch to automatic probe cycle.','err');
  await poll();applyEnable();
});

/* ---------- chart controls ---------- */
$('tabs').innerHTML=M.map(function(m){return '<button class="btn" type="button" role="tab" id="tab_'+m.k+'" aria-selected="'+(m.k===S.metric)+'">'+m.n+'</button>';}).join('');
M.forEach(function(m){$('tab_'+m.k).addEventListener('click',function(){
  S.metric=m.k;S.hover=-1;M.forEach(function(o){$('tab_'+o.k).setAttribute('aria-selected',o.k===m.k?'true':'false');});chart();});});
var cv=$('chart');
function hoverAt(e){
  var d=S.d;if(!d)return;var h=d[MM[S.metric].hk]||[],r=cv.getBoundingClientRect(),G=geo(r.width,r.height),N=g('histSize',30);
  var step=(r.width-G.L-G.R)/(N-1),i=h.length-1-Math.round((r.width-G.R-(e.clientX-r.left))/step);
  S.hover=Math.max(0,Math.min(h.length-1,i));chart();
}
cv.addEventListener('pointermove',hoverAt);cv.addEventListener('pointerdown',hoverAt);
cv.addEventListener('pointerleave',function(){S.hover=-1;chart();});
if(window.ResizeObserver)new ResizeObserver(function(){chart();}).observe(cv.parentNode);else window.addEventListener('resize',chart);

/* ---------- static markup built once ---------- */
$('cards').innerHTML=M.map(function(m){return '<article class="card" style="--mc:'+m.c+'" aria-label="'+m.n+'">'+
'<div class="c-h"><span class="c-t">'+m.n+'</span><span class="tag" id="t_'+m.k+'" data-c="off">No data</span></div>'+
'<div class="c-v"><span class="num" id="v_'+m.k+'">--</span><span class="unit">'+m.u+'</span></div>'+
'<div><div class="meter"><i id="g_'+m.k+'"></i></div><div class="mm"><span>'+m.lo+'</span><span>'+m.hi+'</span></div></div>'+
'<div class="c-f"><svg class="spark" viewBox="0 0 100 28" preserveAspectRatio="none" aria-hidden="true"><path id="s_'+m.k+'" d=""/></svg><span class="trend" id="r_'+m.k+'"></span></div></article>';}).join('');
$('dist').innerHTML=SEG.map(function(s,i){return '<i id="sg'+i+'" style="background:'+s[2]+'"></i>';}).join('');
$('leg').innerHTML=SEG.map(function(s,i){return '<div><i style="background:'+s[2]+'"></i>'+s[1]+'<b id="sv'+i+'">--</b></div>';}).join('');

setInterval(poll,1000);setInterval(link,500);
poll();
})();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiData() {
  int startIdx = (g_histIndex - g_histCount + HISTORY_SIZE) % HISTORY_SIZE;

  String json;
  json.reserve(2300);   // one allocation instead of repeated growth
  json = "{";
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
  json += "\"sensorsReady\":" + String(g_histCount > 0 ? "true" : "false") + ",";
  json += "\"dhtOk\":" + String(g_dhtOk ? "true" : "false") + ",";
  json += "\"dhtEver\":" + String(g_dhtEver ? "true" : "false") + ",";
  json += "\"probeBusy\":" + String(probeState != PROBE_IDLE ? "true" : "false") + ",";
  json += "\"obstacleCm\":" + String(OBSTACLE_DISTANCE_CM) + ",";
  json += "\"safetyCm\":" + String(SAFETY_STOP_CM) + ",";
  json += "\"watchdogMs\":" + String(DRIVE_WATCHDOG_MS) + ",";
  json += "\"histSize\":" + String(HISTORY_SIZE) + ",";
  json += "\"histIntervalMs\":" + String(SENSOR_READ_INTERVAL_MS) + ",";
  json += "\"servoMin\":" + String(SERVO_UP_ANGLE) + ",";
  json += "\"servoMax\":" + String(SERVO_DOWN_ANGLE) + ",";
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
  if (!g_running) t_lastSoilProbe = millis();   // first dip happens one full interval after AUTO starts
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
// BOOT DIAGNOSTICS — short text (max 9 chars, fits the 16x2 LCD) for why
// the ESP32 last restarted. "BROWNOUT" = the supply voltage dipped too
// low (the classic symptom of a weak/shared power source).
// ========================================================================
const char* resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWER-ON";
    case ESP_RST_EXT:       return "EXT-RESET";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "CRASH";
    case ESP_RST_INT_WDT:   return "WATCHDOG";
    case ESP_RST_TASK_WDT:  return "WATCHDOG";
    case ESP_RST_WDT:       return "WATCHDOG";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    default:                return "OTHER";
  }
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
  lcdBegin();
  lcd.setCursor(0, 0);
  lcd.print("Aestra Rover");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");

  // Report why we (re)started — on Serial and on the LCD for ~1.5s.
  const char* why = resetReasonText(esp_reset_reason());
  Serial.print("Reset reason: ");
  Serial.println(why);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Reset: ");
  lcd.print(why);
  lcd.setCursor(0, 1);
  lcd.print("Booting...");
  delay(1500);

  WiFi.persistent(false);

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

  WiFi.setSleep(WIFI_PS_NONE);   // must come after WiFi.mode()/begin(), otherwise it is ignored

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
