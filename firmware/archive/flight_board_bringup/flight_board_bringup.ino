/* ============================================================================
 *  FLIGHT BOARD BRING-UP  --  ESP32-A        (Path B, stage 1)
 * ----------------------------------------------------------------------------
 *  Answers the questions the ArduPilot board definition cannot be written
 *  without:
 *
 *    1. Is the IMU a genuine MPU-9250 (WHO_AM_I 0x71, has an AK8963 compass)
 *       or an MPU-6500 (0x70, no compass) or an MPU-6050 (0x68)?
 *    2. What address is the barometer on, 0x76 or 0x77, and is it a BMP280
 *       or a BME280?
 *    3. Does the battery divider on IO36 read correctly, and what scale
 *       factor does ArduPilot need?
 *    4. Do all four ESC outputs drive the right motor?
 *
 *  Wiring, from the perfboard plan:
 *    I2C      SDA = GPIO27   SCL = GPIO14
 *    IMU INT  GPIO32
 *    VBAT     GPIO36 (ADC1) via divider R1/R2, R2 = 10k
 *    ESCs     ESC1 = 23  ESC2 = 22  ESC3 = 21  ESC4 = 19
 *    UART     RX 34 / TX 13  to ESP32-B (not used here)
 *
 *  >>> PROPELLERS OFF for anything in the ESC section. <<<
 *
 *  Serial: 115200, line ending "Newline". Line-buffered.
 *  Run 'all' first, then paste the BOARD FILE SUMMARY back.
 * ========================================================================= */

#include <Arduino.h>
#include <Wire.h>

/* ------------------------------------------------------------------ PINS */
static const int PIN_SDA = 27;
static const int PIN_SCL = 14;
static const int PIN_IMU_INT = 32;
static const int PIN_VBAT = 36;
static const int ESC_PINS[4] = {23, 22, 21, 19};
static const int ESC_CH[4]   = {4, 5, 6, 7};

/* ------------------------------------------------- battery divider ------
 * Perfboard plan: R1 = 33k for 3S, 47k for 4S; R2 = 10k.
 * ratio = (R1 + R2) / R2  ->  4.30 for 3S, 5.70 for 4S.
 * 'cal <volts>' measures the pack with a multimeter and corrects the scale.
 * ---------------------------------------------------------------------- */
float divRatio = 5.70f;              // change to 4.30f if you fitted 33k
float calScale = 1.00f;

/* ---------------------------------------------------------------- STATE */
int   imuAddr = -1, imuWho = -1;
int   magAddr = -1, magWho = -1;
int   baroAddr = -1, baroId = -1;
bool  escAttached = false;
String line;

/* ====================================================================== */
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static int i2cRead8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)addr, 1) != 1) return -1;
  return Wire.read();
}

static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

/* ---------------------------------------------------------------- SCAN */
static void scanBus() {
  Serial.println(F("\n=== I2C SCAN  (SDA 27 / SCL 14) ==="));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (!i2cPresent(a)) continue;
    found++;
    Serial.printf("  0x%02X  ", a);
    switch (a) {
      case 0x68: case 0x69: Serial.println(F("IMU (MPU-6050/6500/9250)")); break;
      case 0x0C:            Serial.println(F("AK8963 magnetometer")); break;
      case 0x76: case 0x77: Serial.println(F("BMP280 / BME280 barometer")); break;
      case 0x1E:            Serial.println(F("HMC5883 magnetometer")); break;
      case 0x3C:            Serial.println(F("OLED display")); break;
      default:              Serial.println(F("unknown")); break;
    }
  }
  if (!found) {
    Serial.println(F("  NOTHING FOUND."));
    Serial.println(F("  Check: 3V3 to both sensors, GND common, SDA/SCL not swapped,"));
    Serial.println(F("  and 4.7k pull-ups to 3V3 on SDA and SCL if the breakout has none."));
  }
  Serial.printf("  %d device(s)\n\n", found);
}

/* ----------------------------------------------------------------- IMU */
static const char *imuName(int who) {
  switch (who) {
    case 0x71: return "MPU-9250   (has AK8963 compass)";
    case 0x73: return "MPU-9255   (has AK8963 compass)";
    case 0x70: return "MPU-6500   (NO compass)";
    case 0x68: return "MPU-6050   (NO compass, older part)";
    case 0x12: return "ICM-20689";
    case 0x11: return "ICM-20602";
    default:   return "UNRECOGNISED";
  }
}

static void checkImu() {
  Serial.println(F("=== IMU ==="));
  imuAddr = imuWho = -1;
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    if (!i2cPresent(a)) continue;
    int who = i2cRead8(a, 0x75);                 // WHO_AM_I
    if (who < 0) continue;
    imuAddr = a; imuWho = who;
    Serial.printf("  address  0x%02X\n", a);
    Serial.printf("  WHO_AM_I 0x%02X  ->  %s\n", who, imuName(who));
    break;
  }
  if (imuAddr < 0) { Serial.println(F("  NO IMU FOUND\n")); return; }

  // Wake it and look for the AK8963 behind the bypass
  i2cWrite8(imuAddr, 0x6B, 0x00);                // PWR_MGMT_1 = 0, wake
  delay(50);
  i2cWrite8(imuAddr, 0x37, 0x02);                // INT_PIN_CFG, I2C bypass on
  delay(20);

  magAddr = magWho = -1;
  if (i2cPresent(0x0C)) {
    int wia = i2cRead8(0x0C, 0x00);              // AK8963 WIA, expect 0x48
    magAddr = 0x0C; magWho = wia;
    Serial.printf("  compass  AK8963 at 0x0C, WIA 0x%02X %s\n",
                  wia, wia == 0x48 ? "(good)" : "(UNEXPECTED)");
  } else {
    Serial.println(F("  compass  none behind the bypass"));
    if (imuWho == 0x70)
      Serial.println(F("           expected: an MPU-6500 has no magnetometer."));
    else if (imuWho == 0x71)
      Serial.println(F("           NOT expected for an MPU-9250 -- the module may be"
                       "\n           a relabelled MPU-6500, which is very common."));
  }

  // A quick liveness check: raw accel should change when you tilt the board
  int xh = i2cRead8(imuAddr, 0x3B), xl = i2cRead8(imuAddr, 0x3C);
  int zh = i2cRead8(imuAddr, 0x3F), zl = i2cRead8(imuAddr, 0x40);
  if (xh >= 0 && zh >= 0) {
    int16_t ax = (int16_t)((xh << 8) | xl);
    int16_t az = (int16_t)((zh << 8) | zl);
    Serial.printf("  raw accel  X %6d   Z %6d   (Z near +/-16384 when level)\n", ax, az);
  }
  Serial.println();
}

/* ---------------------------------------------------------------- BARO */
static void checkBaro() {
  Serial.println(F("=== BAROMETER ==="));
  baroAddr = baroId = -1;
  for (uint8_t a = 0x76; a <= 0x77; a++) {
    if (!i2cPresent(a)) continue;
    int id = i2cRead8(a, 0xD0);                  // chip id register
    baroAddr = a; baroId = id;
    Serial.printf("  address  0x%02X\n", a);
    Serial.printf("  chip id  0x%02X  ->  ", id);
    if      (id == 0x58) Serial.println(F("BMP280"));
    else if (id == 0x60) Serial.println(F("BME280 (has humidity)"));
    else if (id == 0x61) Serial.println(F("BME680"));
    else                 Serial.println(F("UNRECOGNISED"));
    break;
  }
  if (baroAddr < 0) Serial.println(F("  NO BAROMETER FOUND"));
  Serial.println();
}

/* ------------------------------------------------------------- BATTERY */
static float readVbat() {
  uint32_t acc = 0;
  for (int i = 0; i < 32; i++) { acc += analogReadMilliVolts(PIN_VBAT); delay(2); }
  float pinV = (acc / 32.0f) / 1000.0f;
  return pinV * divRatio * calScale;
}

static void checkBatt() {
  uint32_t acc = 0;
  for (int i = 0; i < 32; i++) { acc += analogReadMilliVolts(PIN_VBAT); delay(2); }
  float pinV = (acc / 32.0f) / 1000.0f;
  Serial.println(F("=== BATTERY SENSE (IO36) ==="));
  Serial.printf("  at the pin   %.3f V\n", pinV);
  Serial.printf("  divider      %.2f  x cal %.3f\n", divRatio, calScale);
  Serial.printf("  pack         %.2f V\n", pinV * divRatio * calScale);
  if (pinV > 3.1f)
    Serial.println(F("  !! near the ADC ceiling -- the divider ratio is too low for"
                     "\n     this pack. Raise R1 before trusting any reading."));
  if (pinV < 0.05f)
    Serial.println(F("  !! almost zero. Pack not connected, or the divider is open."));
  Serial.println(F("  Measure the pack with a multimeter, then: cal <volts>\n"));
}

static void calibrateBatt(float actual) {
  uint32_t acc = 0;
  for (int i = 0; i < 32; i++) { acc += analogReadMilliVolts(PIN_VBAT); delay(2); }
  float pinV = (acc / 32.0f) / 1000.0f;
  if (pinV < 0.05f) { Serial.println(F("  pin reads ~0 V, connect the pack first")); return; }
  calScale = actual / (pinV * divRatio);
  Serial.printf("  calScale = %.4f   -> pack now reads %.2f V\n",
                calScale, pinV * divRatio * calScale);
  Serial.printf("  ArduPilot BATT_VOLT_MULT = %.4f\n\n", divRatio * calScale);
}

/* ----------------------------------------------------------------- ESC */
static uint32_t usToDuty(int us) { return (uint32_t)(((uint32_t)us * 65535UL) / 20000UL); }

static void escAttach() {
  if (escAttached) return;
  for (int i = 0; i < 4; i++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(ESC_PINS[i], 50, 16, ESC_CH[i]);
#else
    ledcSetup(ESC_CH[i], 50, 16); ledcAttachPin(ESC_PINS[i], ESC_CH[i]);
#endif
  }
  escAttached = true;
}

static void escWrite(int i, int us) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ESC_PINS[i], usToDuty(us));
#else
  ledcWrite(ESC_CH[i], usToDuty(us));
#endif
}

static void escAllMin() { for (int i = 0; i < 4; i++) escWrite(i, 1000); }

// Short bump of each motor in turn. For full calibration and motor ID use
// esc_tuner_v2.ino -- this is only a "does the wire work" check.
static void escBump() {
  escAttach();
  escAllMin();
  Serial.println(F("\n=== ESC BUMP TEST -- PROPELLERS OFF ==="));
  Serial.println(F("  Each motor gets 1150 us for 1.2 s. Any key aborts.\n"));
  Serial.println(F("  Press any key within 3 s to cancel..."));
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (Serial.available()) { while (Serial.available()) Serial.read();
      Serial.println(F("  cancelled\n")); return; }
    delay(10);
  }
  for (int i = 0; i < 4; i++) {
    Serial.printf("  ESC%d (GPIO%d) ...\n", i + 1, ESC_PINS[i]);
    escWrite(i, 1150);
    uint32_t t1 = millis();
    while (millis() - t1 < 1200) {
      if (Serial.available()) { while (Serial.available()) Serial.read();
        escAllMin(); Serial.println(F("  ABORTED\n")); return; }
      delay(5);
    }
    escWrite(i, 1000);
    delay(700);
  }
  escAllMin();
  Serial.println(F("  All four done. A silent motor = that wire, that ESC, or an"));
  Serial.println(F("  uncalibrated ESC. Use esc_tuner_v2.ino to calibrate.\n"));
}

/* ------------------------------------------------------------- SUMMARY */
static void summary() {
  Serial.println(F("\n================ BOARD FILE SUMMARY ================"));
  Serial.println(F("  Paste this back; it is what the ArduPilot board"));
  Serial.println(F("  definition needs."));
  Serial.println(F("  ----------------------------------------------"));
  Serial.printf("  I2C pins        SDA %d, SCL %d\n", PIN_SDA, PIN_SCL);
  if (imuAddr >= 0) {
    Serial.printf("  IMU             0x%02X, WHO_AM_I 0x%02X\n", imuAddr, imuWho);
    Serial.printf("  IMU type        %s\n", imuName(imuWho));
  } else Serial.println(F("  IMU             NOT FOUND"));
  if (magAddr >= 0) Serial.printf("  Compass         AK8963 0x0C, WIA 0x%02X\n", magWho);
  else              Serial.println(F("  Compass         none  -> no yaw reference, "
                                     "LOITER and RTL will not work"));
  if (baroAddr >= 0) Serial.printf("  Barometer       0x%02X, chip id 0x%02X\n", baroAddr, baroId);
  else               Serial.println(F("  Barometer       NOT FOUND -> no ALT_HOLD"));
  Serial.printf("  Battery pin     GPIO%d (ADC1)\n", PIN_VBAT);
  Serial.printf("  BATT_VOLT_MULT  %.4f\n", divRatio * calScale);
  Serial.printf("  ESC outputs     %d, %d, %d, %d\n",
                ESC_PINS[0], ESC_PINS[1], ESC_PINS[2], ESC_PINS[3]);
  Serial.println(F("  ====================================================\n"));

  if (magAddr < 0)
    Serial.println(F("  NOTE: with no compass you can fly STABILIZE and ALT_HOLD,\n"
                     "        but not LOITER, POSHOLD or RTL. For an indoor warehouse\n"
                     "        demo that may be acceptable -- decide now, not later.\n"));
}

/* ---------------------------------------------------------------- HELP */
static void help() {
  Serial.println(F("\n---------- FLIGHT BOARD BRING-UP (ESP32-A) ----------"));
  Serial.println(F("  all          scan + imu + baro + batt + summary"));
  Serial.println(F("  scan         I2C bus scan"));
  Serial.println(F("  imu          identify the IMU and its compass"));
  Serial.println(F("  baro         identify the barometer"));
  Serial.println(F("  batt         read the battery divider"));
  Serial.println(F("  cal <volts>  calibrate the divider against a multimeter"));
  Serial.println(F("  ratio <n>    divider ratio: 4.30 for 3S (33k), 5.70 for 4S (47k)"));
  Serial.println(F("  esc          bump each motor in turn -- PROPELLERS OFF"));
  Serial.println(F("  sum          print the board file summary again"));
  Serial.println(F("  ?            this help"));
  Serial.println(F("-----------------------------------------------------\n"));
}

static void handle(String in) {
  in.trim();
  if (!in.length()) return;
  if      (in == "?" || in == "help") help();
  else if (in == "scan") scanBus();
  else if (in == "imu")  checkImu();
  else if (in == "baro") checkBaro();
  else if (in == "batt") checkBatt();
  else if (in == "esc")  escBump();
  else if (in == "sum")  summary();
  else if (in == "all")  { scanBus(); checkImu(); checkBaro(); checkBatt(); summary(); }
  else if (in.startsWith("cal ")) calibrateBatt(in.substring(4).toFloat());
  else if (in.startsWith("ratio ")) {
    float r = in.substring(6).toFloat();
    if (r > 1.0f && r < 20.0f) { divRatio = r; Serial.printf("  divider ratio = %.2f\n", r); }
    else Serial.println(F("  usage: ratio 5.70"));
  }
  else Serial.printf("  unknown '%s'  (? for help)\n", in.c_str());
}

/* --------------------------------------------------------------- SETUP */
void setup() {
  for (int i = 0; i < 4; i++) { pinMode(ESC_PINS[i], OUTPUT); digitalWrite(ESC_PINS[i], LOW); }
  pinMode(PIN_IMU_INT, INPUT);

  Serial.begin(115200);
  delay(500);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                  // slow and reliable for bring-up

  Serial.println(F("\n\n=== ESP32-A FLIGHT BOARD BRING-UP ==="));
  Serial.printf("I2C SDA %d / SCL %d   VBAT GPIO%d   ESC %d/%d/%d/%d\n",
                PIN_SDA, PIN_SCL, PIN_VBAT,
                ESC_PINS[0], ESC_PINS[1], ESC_PINS[2], ESC_PINS[3]);
  Serial.println(F("Type 'all' to run everything. PROPELLERS OFF before 'esc'."));
  help();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (line.length()) { handle(line); line = ""; } }
    else if (line.length() < 24) line += c;
  }
}
