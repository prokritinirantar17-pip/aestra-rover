# AESTRA Rover — Complete Pinout and Wiring Layout

## 1. ESP32 Pin Connections

| ESP32 Pin | Connected Component  | Connection               |
| --------- | -------------------- | ------------------------ |
| GPIO4     | DHT11                | DATA                     |
| GPIO5     | Servo Motor          | SIGNAL                   |
| GPIO14    | L298N Motor Driver   | IN4                      |
| GPIO18    | Ultrasonic Sensor    | TRIG                     |
| GPIO19    | Ultrasonic Sensor    | ECHO                     |
| GPIO21    | I²C LCD              | SDA                      |
| GPIO22    | I²C LCD              | SCL                      |
| GPIO25    | L298N Motor Driver   | IN1                      |
| GPIO26    | L298N Motor Driver   | IN2                      |
| GPIO27    | L298N Motor Driver   | IN3                      |
| GPIO34    | Soil Moisture Sensor | AO                       |
| GPIO35    | LDR Circuit          | Voltage-divider junction |

## 2. Battery and Power Switch

| Component    | Terminal        | Connection           |
| ------------ | --------------- | -------------------- |
| Battery      | Positive (+)    | Power switch input   |
| Battery      | Negative (−)    | Buck converter IN−   |
| Battery      | Negative (−)    | Breadboard GND rail  |
| Power Switch | Input terminal  | Battery positive (+) |
| Power Switch | Output terminal | Buck converter IN+   |
| Power Switch | Output terminal | L298N 12V / VS       |

## 3. Buck Converter

| Buck Converter Terminal | Connection           |
| ----------------------- | -------------------- |
| IN+                     | Power switch output  |
| IN−                     | Battery negative (−) |
| OUT+                    | Breadboard +5V rail  |
| OUT−                    | Breadboard GND rail  |

## 4. L298N Motor Driver

### Power Connections

| L298N Terminal | Connection          |
| -------------- | ------------------- |
| 12V / VS       | Power switch output |
| GND            | Breadboard GND rail |
| ENA            | Jumper installed    |
| ENB            | Jumper installed    |

### Control Connections

| L298N Terminal | ESP32 Pin |
| -------------- | --------- |
| IN1            | GPIO25    |
| IN2            | GPIO26    |
| IN3            | GPIO27    |
| IN4            | GPIO14    |

### Motor Connections

| L298N Output  | Connected Component |
| ------------- | ------------------- |
| OUT1 and OUT2 | Left motor pair     |
| OUT3 and OUT4 | Right motor pair    |

## 5. Power Distribution

### +5V Power Rail

| Component      | Power Connection |
| -------------- | ---------------- |
| ESP32          | VIN / 5V         |
| I²C LCD        | VCC              |
| Servo Motor    | VCC              |
| Buck Converter | OUT+             |

### +3.3V Power Rail

| Component            | Power Connection             |
| -------------------- | ---------------------------- |
| DHT11                | VCC                          |
| Soil Moisture Sensor | VCC                          |
| LDR Circuit          | Voltage-divider supply       |
| Ultrasonic Sensor    | VCC                          |
| ESP32                | 3V3 output supplies the rail |

### Common GND Power Rail

| Component            | Ground Connection |
| -------------------- | ----------------- |
| Battery              | Negative (−)      |
| Buck Converter       | OUT−              |
| ESP32                | GND               |
| DHT11                | GND               |
| Servo Motor          | GND               |
| Ultrasonic Sensor    | GND               |
| I²C LCD              | GND               |
| Soil Moisture Sensor | GND               |
| LDR Circuit          | Ground connection |
| L298N Motor Driver   | GND               |

## 6. Complete Component Pinout Summary

| Component            | Signal Pin(s)                                          | Power                                 | Ground               |
| -------------------- | ------------------------------------------------------ | ------------------------------------- | -------------------- |
| ESP32                | —                                                      | VIN / 5V rail; 3V3 output             | Common GND           |
| DHT11                | DATA → GPIO4                                           | +3.3V                                 | Common GND           |
| Servo Motor          | SIGNAL → GPIO5                                         | +5V                                   | Common GND           |
| Ultrasonic Sensor    | TRIG → GPIO18; ECHO → GPIO19                           | +3.3V                                 | Common GND           |
| I²C LCD              | SDA → GPIO21; SCL → GPIO22                             | +5V                                   | Common GND           |
| Soil Moisture Sensor | AO → GPIO34                                            | +3.3V                                 | Common GND           |
| LDR Circuit          | Junction → GPIO35                                      | +3.3V divider supply                  | Common GND           |
| L298N Motor Driver   | IN1 → GPIO25; IN2 → GPIO26; IN3 → GPIO27; IN4 → GPIO14 | Switched battery positive to 12V / VS | Common GND           |
| Left Motor Pair      | —                                                      | L298N OUT1 / OUT2                     | Through motor driver |
| Right Motor Pair     | —                                                      | L298N OUT3 / OUT4                     | Through motor driver |
