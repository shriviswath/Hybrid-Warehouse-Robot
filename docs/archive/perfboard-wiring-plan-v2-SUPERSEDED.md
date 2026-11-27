# SUPERSEDED — perfboard wiring plan (v2, Sep 2026)

> **Do not wire from this document.** The GPIO assignment here (ENA on IO23, IN1 on
> IO21, rack servo on IO16, and ESP32-A as the flight board) was replaced in November
> 2026. The current assignment is the
> [README pinout table](../../README.md#pinout-authoritative). Kept for the layout
> reasoning and the ESP32 erratum note at the end.

Grid: columns 1–33 as printed (col 1 = edge-header column); rows 1–54 from the printed
bottom "A" row. Parts: ESP32-A rows 37/47, ESP32-B rows 7/17, both cols 10–24.
MPU-9250 row 30, cols 13–22. BMP280 col 26, rows 24–29. All traces solder side; the
only crossing is one insulated component-side jumper (J1 at c22 r36 → J2 at c21 r18).

## Edge header (pin p is at row 48−p, so p1 is next to ESP32-A)

| Pin | Net |
|---|---|
| p2–p5 | ESC1–4 ← A-IO23 / IO22 / IO21 / IO19 |
| p6 | GND (ESC reference) |
| p9 / p10 | +5V / GND for ESP32-A |
| p13 | VBAT raw → divider (R1 33k for 3S, 47k for 4S; R2 10k; 100n) → A-IO36 |
| p17 | GND |
| p29 / p30 | GND / +5V for ESP32-B |
| p32 | GND (drive/servo) |
| p33–p38 | ENA IO23, ENB IO22, IN1 IO21, IN2 IO19, IN3 IO18, IN4 IO17 (ESP32-B) |
| p39 / p40 | Servo1 rack IO16, Servo2 gripper IO4 (ESP32-B) |
| p1, p7–8, p11–12, p14–16, p18–28, p31 | spare |

## Rail routing

- IMU and baro power: 3V3 from A's 3V3 pin down col 25. Sensor GND from A's top GND
  pin via row 48 and col 27; col 27 continues to row 6 and ties to B's GND pin.
- UART: B-IO13 → A-IO34 down col 12 with a 1k at rows 18–22. A-IO13 → B-IO34 is the
  J1→J2 jumper with a 1k inline.

## Note still worth keeping

I2C was moved to IO27/IO14 and UART RX to IO34 because **GPIO36 and GPIO39 are
unsuitable as UART RX** — the ESP32 input-glitch erratum. That constraint survives
into the current build.
