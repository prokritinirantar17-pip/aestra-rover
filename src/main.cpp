/*
  ========================================================================
  AESTRA — Autonomous Smart Agricultural Monitoring Rover
  ========================================================================
  Built for: ESP32 Dev Board (USB Type-C)

  Functionalities implemented (per project PDF):
    1. Autonomous Navigation & Obstacle Avoidance (HC-SR04 + L298N motors)
    2. Intelligent Environmental & Soil Sensing (Soil moisture + LDR light
       sensor, with a servo that lowers a probe arm into the soil)
    3. Wireless Telemetry & IoT Dashboard Monitoring (ESP32 Wi-Fi web page
       that shows live sensor readings, auto-refreshing)

  NOT implemented (per your instruction — excluded from the final build):
    - Buzzer, Push Buttons, Potentiometer, manual/auto toggle

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

  L298N channel layout (one driver, two channels, four motors total —
  left-front + left-back share one channel, right-front + right-back
  share the other):
    IN1/IN2  -> OUT1/OUT2 -> LEFT side motors (front + back, wired parallel)
    IN3/IN4  -> OUT3/OUT4 -> RIGHT side motors (front + back, wired parallel)

  If the rover drives backward when you meant forward, or spins the wrong
  way, just swap the HIGH/LOW pair for that side in the functions below —
  this is normal and just depends on how the motor wires were connected.
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
// Mode A (default): ESP32 creates its OWN Wi-Fi hotspot. Connect your
// phone/laptop to this network and open the IP shown on Serial Monitor
// (usually 192.168.4.1) to see the live dashboard. No home router needed
// — good for a demo table with no internet.
#define USE_ACCESS_POINT_MODE false

const char* AP_SSID     = "Aestra-Rover";
const char* AP_PASSWORD = "aestra123";   // must be at least 8 characters

// Mode B: connect to your home/school Wi-Fi instead. Set the flag above
// to false and fill these in.
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

// I2C LCD address is commonly 0x27 or 0x3F — if the screen stays blank,
// try changing this to 0x3F.
#define LCD_ADDRESS    0x27
#define LCD_COLS       16
#define LCD_ROWS       2

// ---------------------------------------------------------------------
// TUNABLE CONSTANTS — calibrate these to your own hardware
// ---------------------------------------------------------------------
const int OBSTACLE_DISTANCE_CM   = 20;    // stop/turn if something is closer than this
const unsigned long AVOID_BACK_MS  = 350; // how long to reverse when avoiding
const unsigned long AVOID_TURN_MS  = 500; // how long to turn when avoiding

const unsigned long SENSOR_READ_INTERVAL_MS = 2000;  // soil/light/DHT read rate
const unsigned long LCD_SWITCH_INTERVAL_MS  = 3000;  // how often LCD screen flips
const unsigned long DISTANCE_CHECK_INTERVAL_MS = 100; // obstacle check rate

const unsigned long SOIL_PROBE_INTERVAL_MS = 30000;  // how often to dip the probe
const unsigned long SOIL_PROBE_DWELL_MS    = 2000;   // how long probe stays down

// Soil sensor raw ADC calibration (12-bit ESP32 ADC: 0-4095).
// Many resistive soil sensors read HIGH when dry and LOW when wet —
// dip the probe in a dry vs. wet sample and update these two numbers.
const int SOIL_RAW_DRY = 4095;
const int SOIL_RAW_WET = 1500;

// LDR voltage-divider calibration (LDR to 3.3V, 10k resistor to GND,
// junction to GPIO35). Reading rises as light increases with this wiring.
const int LDR_RAW_DARK   = 300;
const int LDR_RAW_BRIGHT = 4095;

// Servo angles for the soil probe arm
const int SERVO_UP_ANGLE   = 0;    // resting / traveling position
const int SERVO_DOWN_ANGLE = 90;   // lowered into the soil

// ---------------------------------------------------------------------
// GLOBAL OBJECTS
// ---------------------------------------------------------------------
DHT dht(DHT_PIN, DHT_TYPE);
Servo probeServo;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// ---------------------------------------------------------------------
// LIVE SENSOR STATE (shared between loop() logic, LCD, and web dashboard)
// ---------------------------------------------------------------------
float g_distanceCm   = -1;
int   g_soilPercent  = 0;
int   g_lightPercent = 0;
float g_temperatureC = 0;
float g_humidityPct  = 0;
bool  g_probeDown     = false;
bool  g_obstacleAvoiding = false;

// Timers
unsigned long t_lastDistanceCheck = 0;
unsigned long t_lastSensorRead    = 0;
unsigned long t_lastLcdSwitch     = 0;
unsigned long t_lastSoilProbe     = 0;
unsigned long t_lastWifiCheck     = 0;
bool lcdShowingScreenA = true;

const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;

// ========================================================================
// MOTOR CONTROL
// (ENA/ENB are jumper-capped ON, so these pins only set direction —
//  there is no speed/PWM control on this build.)
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

// Pivot turn: spin left side backward, right side forward -> turns left
void motorsTurnLeft() {
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
}

// Pivot turn: spin left side forward, right side backward -> turns right
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

  // 30000us timeout ~ 5m range, avoids blocking forever if no echo returns
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    return -1;  // no echo received (out of range or misread)
  }
  return duration * 0.0343 / 2.0;  // speed of sound conversion -> cm
}

// ========================================================================
// OBSTACLE AVOIDANCE (Functionality 1)
// ========================================================================
void handleObstacleAvoidance() {
  if (g_distanceCm > 0 && g_distanceCm < OBSTACLE_DISTANCE_CM) {
    g_obstacleAvoiding = true;

    motorsStop();
    delay(150);

    motorsBackward();
    delay(AVOID_BACK_MS);

    motorsStop();
    delay(100);

    motorsTurnRight();
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
}

// Periodically pause the rover, lower the probe arm into the soil, take a
// fresh moisture reading, then raise it again.
void handleSoilProbeCycle() {
  unsigned long now = millis();
  if (now - t_lastSoilProbe < SOIL_PROBE_INTERVAL_MS) return;
  t_lastSoilProbe = now;

  motorsStop();

  g_probeDown = true;
  probeServo.write(SERVO_DOWN_ANGLE);
  delay(SOIL_PROBE_DWELL_MS);

  int rawSoil = analogRead(SOIL_PIN);
  g_soilPercent = constrain(map(rawSoil, SOIL_RAW_DRY, SOIL_RAW_WET, 0, 100), 0, 100);

  probeServo.write(SERVO_UP_ANGLE);
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
    lcd.print("cm");

    lcd.setCursor(0, 1);
    lcd.print("Soil:");
    lcd.print(g_soilPercent);
    lcd.print("%");
    if (g_probeDown) lcd.print(" DOWN");
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
// WEB DASHBOARD (Functionality 3)
// ========================================================================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta http-equiv='refresh' content='2'>";
  html += "<title>Aestra Rover Dashboard</title>";
  html += "<style>body{font-family:Arial, sans-serif; background:#f4f6f0; color:#2c3e2d; text-align:center; padding:30px;}";
  html += "h1{color:#3d6b35;} .card{background:#fff; border-radius:12px; box-shadow:0 2px 8px rgba(0,0,0,0.1); display:inline-block; padding:20px 40px; margin:10px;}";
  html += ".value{font-size:28px; font-weight:bold;} .label{font-size:14px; color:#666;}</style>";
  html += "</head><body>";
  html += "<h1>Aestra Rover — Live Telemetry</h1>";

  html += "<div class='card'><div class='label'>Distance to Obstacle</div><div class='value'>";
  html += (g_distanceCm > 0 ? String(g_distanceCm, 0) : "--");
  html += " cm</div></div>";

  html += "<div class='card'><div class='label'>Soil Moisture</div><div class='value'>";
  html += String(g_soilPercent) + " %</div></div>";

  html += "<div class='card'><div class='label'>Ambient Light</div><div class='value'>";
  html += String(g_lightPercent) + " %</div></div>";

  html += "<div class='card'><div class='label'>Air Temperature</div><div class='value'>";
  html += String(g_temperatureC, 1) + " &deg;C</div></div>";

  html += "<div class='card'><div class='label'>Humidity</div><div class='value'>";
  html += String(g_humidityPct, 0) + " %</div></div>";

  html += "<div class='card'><div class='label'>Soil Probe</div><div class='value'>";
  html += (g_probeDown ? "LOWERED" : "UP");
  html += "</div></div>";

  html += "<div class='card'><div class='label'>Status</div><div class='value'>";
  html += (g_obstacleAvoiding ? "Avoiding obstacle" : "Navigating");
  html += "</div></div>";

  html += "<p style='color:#999; font-size:12px;'>Page auto-refreshes every 2 seconds.</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ========================================================================
// SETUP
// ========================================================================
void setup() {
  Serial.begin(9600);

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

  Wire.begin(21, 22); // SDA, SCL
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Aestra Rover");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");

  // --- Wi-Fi ---
  WiFi.persistent(false);          // don't stash stale credentials in flash
  WiFi.setSleep(WIFI_PS_NONE);     // keep radio awake so the web dashboard stays reachable

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
  server.begin();

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

  unsigned long now = millis();

  // 1) Distance check + obstacle avoidance (highest priority, most frequent)
  if (now - t_lastDistanceCheck >= DISTANCE_CHECK_INTERVAL_MS) {
    t_lastDistanceCheck = now;
    g_distanceCm = readDistanceCm();
    handleObstacleAvoidance();
  }

  // 2) Environmental sensing (soil, light, temp, humidity)
  if (now - t_lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    t_lastSensorRead = now;
    readEnvironmentSensors();
  }

  // 3) Periodic soil probe dip
  handleSoilProbeCycle();

  // 4) LCD refresh
  updateLcd();
}
