# aestra-rover


Autonomous smart agricultural monitoring rover built on an ESP32 (PlatformIO, Arduino framework).

## Pin connections

| ESP32 pin | Connected to |
|-----------|--------------|
| GPIO4 | DHT11 DATA |
| GPIO5 | Servo SIGNAL |
| GPIO14 | L298N IN4 |
| GPIO18 | Ultrasonic TRIG |
| GPIO19 | Ultrasonic ECHO |
| GPIO21 | LCD SDA |
| GPIO22 | LCD SCL |
| GPIO25 | L298N IN1 |
| GPIO26 | L298N IN2 |
| GPIO27 | L298N IN3 |
| GPIO34 | Soil moisture AO |
| GPIO35 | LDR junction |

## Power

```
ESP32
├── 3V3 → 3.3V rail: DHT11 VCC, Soil VCC, LDR, Ultrasonic VCC
├── VIN → 5V rail: LCD VCC, Servo VCC
└── GND → common GND
```

The 3.3V and 5V rails stay completely separate.

## Motor driver (L298N)

- ENA and ENB: jumpers left on
- OUT1/OUT2: left motor pair
- OUT3/OUT4: right motor pair
- VS (+12V): motor battery +
- GND: motor battery − and also connected to ESP32 GND (common ground)