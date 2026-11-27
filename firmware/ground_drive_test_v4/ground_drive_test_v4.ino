/* ============================================================================
 *  GROUND DRIVE TEST  v4   --  PWM path, tuned for the L298N
 * ----------------------------------------------------------------------------
 *  Board : ESP32-B        Driver : HW-095 (L298N)
 *  Pins  : ENA=5   IN1=33  IN2=14  IN3=27  IN4=26  ENB=25
 *
 *  CHANGES IN v4
 *    - IN1 moved GPIO12 -> GPIO33.  GPIO12 is MTDI, a strapping pin read
 *      at reset to pick the internal flash regulator voltage (1.8 or 3.3 V).
 *      Driving it from, or into, an L298N input risks a bad boot and was the
 *      prime suspect for 'left side dead in forward, alive in reverse':
 *      forward asserts IN1, reverse asserts IN2, and only forward failed.
 *      MOVE THE PHYSICAL IN1 WIRE TO D33 BEFORE RUNNING THIS.
 *    - Header/banner pin list corrected: ENA really is GPIO5 (D5).
 *    - New 'p' command: pulses IN1/IN2/IN3/IN4 one at a time with both
 *      enables held HIGH, to find a dead direction line in ten seconds.
 *    - Boot-pin warnings for the strapping pins actually in use.
 *
 *  >>> GPIO5 (ENA) NOTES <<<
 *  GPIO5 is a strapping pin with an internal pull-up; it must read HIGH at
 *  reset.  A bare L298N ENA input is high-impedance so this is normally fine,
 *  but do NOT fit a pulldown on that line.  GPIO5 also emits a short pulse at
 *  boot, so the left enable flickers active on every power-up and reset --
 *  keep the wheels off the bench when powering on.
 *
 *  WHY v2 DID NOT WORK WHILE minimal_motor_test DID
 *  ------------------------------------------------
 *  minimal_motor_test holds the enable pins at a steady digitalWrite HIGH:
 *  100% duty, DC. v2 drove them with PWM clamped to MAX_DUTY = 145/255 (57%)
 *  at 5 kHz. Two things were working against it:
 *
 *   1. THE CLAMP WAS TOO LOW. 145/255 was sized for an 11.1 V 3S pack to
 *      protect 6 V motors. If your supply is lower than that, or the two
 *      paralleled motors drag more than assumed, 57% simply never reaches
 *      breakaway torque. The sweep topping out at 145 is why you saw the
 *      whole ramp print with no movement.
 *
 *   2. 5 kHz IS FAST FOR AN L298N. It is a bipolar Darlington bridge, not a
 *      MOSFET one. Turn-off storage time runs into tens of microseconds, so
 *      at a 200 us period a short on-pulse is mostly eaten by switching.
 *      Low duties produce far less output than the numbers suggest.
 *      v3 defaults to 1 kHz, where the L298N behaves far better.
 *
 *  CHANGES IN v3
 *    - PWM frequency 5000 -> 1000 Hz, and adjustable live with 'f'
 *    - MAX_DUTY 145 -> 255 (clamp removed; see the warning below)
 *    - MIN_DUTY 55 -> 150
 *    - 'o' runs the exact DC full-on case that minimal_motor_test proved,
 *          so you can compare DC against PWM inside one sketch
 *    - '[' and ']' step a raw duty live to find the real PWM breakaway
 *
 *  >>> VOLTAGE WARNING <<<
 *  MAX_DUTY is now 255, so at full speed the motors see the whole pack
 *  voltage minus the bridge drop. The yellow TT gearmotor is a 3-6 V part.
 *  That is fine for short bench runs -- minimal_motor_test already did
 *  exactly this -- but for sustained driving measure your pack voltage and
 *  set MAX_DUTY to  255 * 6 / (Vpack - 2)  rounded down. On 11.1 V that is
 *  145; on 7.4 V it is 255 (no clamp needed).
 *
 *  Serial monitor: 115200, line ending "No line ending".
 *  Wheels off the ground until direction is confirmed.
 * ========================================================================= */

#include <Arduino.h>

/* ------------------------------------------------------------------ PINS */
static const int PIN_ENA = 5;    // left  side PWM  (D5) -- strapping pin, see notes
static const int PIN_IN1 = 33;   // left  dir A   -- moved off GPIO12 (MTDI)
static const int PIN_IN2 = 14;   // left  dir B   -- emits a pulse at boot
static const int PIN_IN3 = 27;   // right dir A
static const int PIN_IN4 = 26;   // right dir B
static const int PIN_ENB = 25;   // right side PWM

/* ------------------------------------------------------------ PWM TUNING */
static const int PWM_BITS   = 8;
static const int PWM_MAX_HW = 255;

static const int FREQ_TABLE[] = {500, 1000, 2000, 5000, 10000};
static const int FREQ_N = sizeof(FREQ_TABLE) / sizeof(FREQ_TABLE[0]);
int freqIdx = 1;                       // start at 1000 Hz

int maxDuty = 255;                     // see the voltage warning above
int minDuty = 150;                     // raise/lower after the 'b' sweep

static const int RAMP_STEP = 8;
static const uint32_t CMD_TIMEOUT_MS = 2500;

static const bool INVERT_LEFT  = false;
static const bool INVERT_RIGHT = false;

/* ----------------------------------------------------------------- STATE */
enum Side { LEFT = 0, RIGHT = 1 };
static const int LEDC_CH[2] = {0, 1};
static const int EN_PIN[2]  = {PIN_ENA, PIN_ENB};

int  targetSigned[2] = {0, 0};
int  curDuty[2]      = {0, 0};
int  curDir[2]       = {0, 0};
int  speedPct        = 80;
int  rawDuty         = 0;              // for the [ ] live duty probe
bool rawMode         = false;
bool latchMode       = false;
bool pwmAttached     = false;
uint32_t lastCmdMs = 0, lastRampMs = 0;

/* ------------------------------------------------------------ LEDC SHIMS */
static void pwmAttach() {
  if (pwmAttached) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_ENA, FREQ_TABLE[freqIdx], PWM_BITS, LEDC_CH[0]);
  ledcAttachChannel(PIN_ENB, FREQ_TABLE[freqIdx], PWM_BITS, LEDC_CH[1]);
#else
  ledcSetup(LEDC_CH[0], FREQ_TABLE[freqIdx], PWM_BITS); ledcAttachPin(PIN_ENA, LEDC_CH[0]);
  ledcSetup(LEDC_CH[1], FREQ_TABLE[freqIdx], PWM_BITS); ledcAttachPin(PIN_ENB, LEDC_CH[1]);
#endif
  pwmAttached = true;
}

static void pwmDetach() {
  if (!pwmAttached) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(PIN_ENA); ledcDetach(PIN_ENB);
#else
  ledcDetachPin(PIN_ENA); ledcDetachPin(PIN_ENB);
#endif
  pinMode(PIN_ENA, OUTPUT); digitalWrite(PIN_ENA, LOW);
  pinMode(PIN_ENB, OUTPUT); digitalWrite(PIN_ENB, LOW);
  pwmAttached = false;
}

static void pwmWrite_(int pin, int ch, int duty) {
  if (!pwmAttached) return;
  duty = constrain(duty, 0, PWM_MAX_HW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;  ledcWrite(pin, duty);
#else
  (void)pin; ledcWrite(ch,  duty);
#endif
}

/* ----------------------------------------------------------- LOW LEVEL -- */
static void setDirPins(Side s, int dir) {
  bool inv = (s == LEFT) ? INVERT_LEFT : INVERT_RIGHT;
  if (inv) dir = -dir;
  int a = (s == LEFT) ? PIN_IN1 : PIN_IN3;
  int b = (s == LEFT) ? PIN_IN2 : PIN_IN4;
  if (dir > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW);  }
  else if (dir < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else              { digitalWrite(a, LOW);  digitalWrite(b, LOW);  }
}

static void stopAll() {
  rawMode = false;
  for (int s = 0; s < 2; s++) {
    targetSigned[s] = curDuty[s] = curDir[s] = 0;
    pwmWrite_(EN_PIN[s], LEDC_CH[s], 0);
    setDirPins((Side)s, 0);
  }
}

/* ----------------------------------------------------------- RAMP ENGINE */
static int dutyFromPct(int pct) {
  pct = constrain(abs(pct), 0, 100);
  if (!pct) return 0;
  return map(pct, 1, 100, minDuty, maxDuty);
}

static void serviceRamp() {
  if (rawMode) return;
  if (millis() - lastRampMs < 10) return;
  lastRampMs = millis();
  for (int s = 0; s < 2; s++) {
    int wantDir  = (targetSigned[s] > 0) ? 1 : (targetSigned[s] < 0 ? -1 : 0);
    int wantDuty = dutyFromPct(targetSigned[s]);
    if (wantDir != curDir[s] && curDuty[s] > 0) {
      curDuty[s] = max(0, curDuty[s] - RAMP_STEP * 2);
    } else {
      curDir[s] = wantDir;
      if (curDuty[s] < wantDuty)      curDuty[s] = min(wantDuty, curDuty[s] + RAMP_STEP);
      else if (curDuty[s] > wantDuty) curDuty[s] = max(wantDuty, curDuty[s] - RAMP_STEP);
    }
    setDirPins((Side)s, curDir[s]);
    pwmWrite_(EN_PIN[s], LEDC_CH[s], curDuty[s]);
  }
}

static void drive(int l, int r) {
  rawMode = false;
  pwmAttach();
  targetSigned[LEFT]  = constrain(l, -100, 100);
  targetSigned[RIGHT] = constrain(r, -100, 100);
  lastCmdMs = millis();
}

/* ------------------------------------------------ 'o'  DC BYPASS TEST ---
 * Reproduces minimal_motor_test exactly: LEDC detached, enables held HIGH
 * with digitalWrite. If this moves and PWM at 255 does not, the problem is
 * the LEDC path, not the motors or the wiring.
 * ---------------------------------------------------------------------- */
static void dcBypass() {
  stopAll();
  pwmDetach();
  Serial.println(F("\n=== DC BYPASS (no PWM, enables held HIGH) ==="));
  Serial.println(F("  This is exactly what minimal_motor_test does."));

  Serial.println(F("  BOTH forward, 2 s ..."));
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_ENA, HIGH); digitalWrite(PIN_ENB, HIGH);
  delay(2000);

  digitalWrite(PIN_ENA, LOW); digitalWrite(PIN_ENB, LOW);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN3, LOW);
  delay(800);

  Serial.println(F("  BOTH reverse, 2 s ..."));
  digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
  digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
  digitalWrite(PIN_ENA, HIGH); digitalWrite(PIN_ENB, HIGH);
  delay(2000);

  digitalWrite(PIN_ENA, LOW); digitalWrite(PIN_ENB, LOW);
  digitalWrite(PIN_IN2, LOW); digitalWrite(PIN_IN4, LOW);

  pwmAttach();
  Serial.println(F("=== DC BYPASS DONE ==="));
  Serial.println(F("  Moved here but not under PWM? -> raise minDuty with ']'"));
  Serial.println(F("  or drop the PWM frequency with 'f'.\n"));
}

/* ------------------------------------------------ 'b'  BREAKAWAY SWEEP -- */
static void breakawaySweep() {
  stopAll();
  pwmAttach();
  Serial.printf("\n=== BREAKAWAY SWEEP at %d Hz, 0 -> 255 ===\n", FREQ_TABLE[freqIdx]);
  Serial.println(F("Press SPACE the moment the wheels turn steadily.\n"));
  setDirPins(LEFT, 1); setDirPins(RIGHT, 1);
  int found = -1;
  for (int d = 40; d <= PWM_MAX_HW; d += 5) {
    pwmWrite_(PIN_ENA, LEDC_CH[0], d);
    pwmWrite_(PIN_ENB, LEDC_CH[1], d);
    Serial.printf("  duty %3d / 255  (%2d%%)\n", d, (d * 100) / PWM_MAX_HW);
    uint32_t t0 = millis();
    while (millis() - t0 < 600) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == ' ' && found < 0) { found = d; Serial.printf("  >>> breakaway at %d\n", d); }
      }
      delay(5);
    }
    if (found > 0 && d >= found + 20) break;
  }
  stopAll();
  if (found > 0) {
    minDuty = min(found + 10, maxDuty - 5);
    Serial.printf("  minDuty set to %d\n", minDuty);
  } else {
    Serial.println(F("  NO BREAKAWAY even at 255/255."));
    Serial.println(F("  Run 'o'. If DC moves and this does not, the PWM path"));
    Serial.println(F("  is the fault -- try 'f' to drop the frequency."));
  }
  Serial.println();
}

/* ------------------------------------------------ '[' ']'  RAW PROBE ---- */
static void rawApply() {
  pwmAttach();
  rawMode = true;
  setDirPins(LEFT, 1); setDirPins(RIGHT, 1);
  pwmWrite_(PIN_ENA, LEDC_CH[0], rawDuty);
  pwmWrite_(PIN_ENB, LEDC_CH[1], rawDuty);
  Serial.printf("raw duty = %3d / 255  (%2d%%)  at %d Hz\n",
                rawDuty, (rawDuty * 100) / PWM_MAX_HW, FREQ_TABLE[freqIdx]);
  lastCmdMs = millis();
}

/* ------------------------------------------------ 'f'  FREQUENCY CYCLE -- */
static void cycleFreq() {
  bool wasRaw = rawMode;
  stopAll();
  pwmDetach();
  freqIdx = (freqIdx + 1) % FREQ_N;
  pwmAttach();
  Serial.printf("PWM frequency now %d Hz\n", FREQ_TABLE[freqIdx]);
  Serial.println(F("  L298N note: lower is better. 500-1000 Hz gives the most"));
  Serial.println(F("  usable low-speed torque; above ~5 kHz it falls off."));
  if (wasRaw) rawApply();
}

/* ---------------------------------------------------- 't' AUTO SEQUENCE */
static void blockingDrive(int l, int r, uint32_t ms, const char *label) {
  Serial.printf("  -> %-20s L=%4d%%  R=%4d%%\n", label, l, r);
  drive(l, r);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { lastCmdMs = millis(); serviceRamp(); delay(2); }
}

static void autoSequence() {
  Serial.println(F("\n=== AUTO SEQUENCE (wheels off the ground) ==="));
  int v = speedPct;
  blockingDrive( v,  0, 1600, "LEFT forward");
  blockingDrive( 0,  0,  500, "pause");
  blockingDrive(-v,  0, 1600, "LEFT reverse");
  blockingDrive( 0,  0,  500, "pause");
  blockingDrive( 0,  v, 1600, "RIGHT forward");
  blockingDrive( 0,  0,  500, "pause");
  blockingDrive( 0, -v, 1600, "RIGHT reverse");
  blockingDrive( 0,  0,  500, "pause");
  blockingDrive( v,  v, 2000, "BOTH forward");
  blockingDrive(-v, -v, 2000, "BOTH reverse");
  blockingDrive( v, -v, 1500, "spin CW");
  blockingDrive(-v,  v, 1500, "spin CCW");
  stopAll();
  Serial.println(F("=== SEQUENCE DONE ===\n"));
}

/* ------------------------------------------------ 'p'  PIN-BY-PIN TEST ---
 * Holds both enables HIGH with plain digitalWrite (no LEDC in the way) and
 * pulses one direction line at a time.  Whichever line produces no motion is
 * the dead GPIO, the dead wire, or the dead half-bridge.  This isolates a
 * single-direction fault without a multimeter.
 * ---------------------------------------------------------------------- */
static void pinByPinTest() {
  stopAll();
  pwmDetach();
  const int  pins[4] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  const char *nm[4]  = {"IN1  LEFT forward ", "IN2  LEFT reverse ",
                        "IN3  RIGHT forward", "IN4  RIGHT reverse"};
  Serial.println(F("\n=== PIN-BY-PIN DIRECTION TEST (wheels off the ground) ==="));
  Serial.println(F("  Both enables held HIGH, one direction pin at a time.\n"));

  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_ENB, HIGH);
  for (int i = 0; i < 4; i++) {
    Serial.printf("  %s  (GPIO%2d)  HIGH for 1.5 s ...\n", nm[i], pins[i]);
    digitalWrite(pins[i], HIGH);
    delay(1500);
    digitalWrite(pins[i], LOW);
    delay(700);
  }
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_ENB, LOW);

  pwmAttach();
  Serial.println(F("\n  Three moved and one did not -> that GPIO, its wire, or"));
  Serial.println(F("  that half of the L298N is the fault."));
  Serial.println(F("  Neither LEFT line moved -> ENA/GPIO5 or the left motor pair."));
  Serial.println(F("  Nothing moved at all   -> supply, ground, or the 5V jumper.\n"));
}

/* ------------------------------------------------------------- REPORTS - */
static void bootPinReport() {
  struct R { int gpio; const char *role; };
  const R rs[6] = {
    {PIN_ENA, "ENA left PWM   "}, {PIN_IN1, "IN1 left dir A "},
    {PIN_IN2, "IN2 left dir B "}, {PIN_IN3, "IN3 right dir A"},
    {PIN_IN4, "IN4 right dir B"}, {PIN_ENB, "ENB right PWM  "},
  };
  Serial.println(F("\n--------------- BOOT PIN REPORT ---------------"));
  Serial.println(F("  pin   role             up  dn  note"));
  for (int i = 0; i < 6; i++) {
    pinMode(rs[i].gpio, INPUT_PULLUP);   delayMicroseconds(800);
    int up = digitalRead(rs[i].gpio);
    pinMode(rs[i].gpio, INPUT_PULLDOWN); delayMicroseconds(800);
    int dn = digitalRead(rs[i].gpio);
    pinMode(rs[i].gpio, OUTPUT); digitalWrite(rs[i].gpio, LOW);
    const char *note = "ok";
    if (dn == 1)      note = "** HELD HIGH - voltage feeding in **";
    else if (up == 0) note = "held low (normal if 10k pulldown fitted)";
    if (rs[i].gpio == 5 && up == 0)
      note = "** GPIO5 PULLED LOW - strapping pin, boot may fail **";
    if (rs[i].gpio == 12 || rs[i].gpio == 15)
      note = "** STRAPPING PIN - move this line to a plain GPIO **";
    Serial.printf("  %-4d  %-15s  %d   %d   %s\n", rs[i].gpio, rs[i].role, up, dn, note);
  }
  Serial.println(F("  ----------------------------------------------\n"));
}

static void printHelp() {
  Serial.println(F("\n--------------- GROUND DRIVE TEST v3 ---------------"));
  Serial.println(F("  p        pulse IN1/IN2/IN3/IN4 one at a time  <- start here"));
  Serial.println(F("  o        DC BYPASS: no PWM, enables HIGH"));
  Serial.println(F("  b        breakaway sweep 0-255, SPACE to mark"));
  Serial.println(F("  [ / ]    raw duty -10 / +10, live, both sides forward"));
  Serial.println(F("  f        cycle PWM frequency 500/1k/2k/5k/10k Hz"));
  Serial.println(F("  1 / 2    left side only / right side only"));
  Serial.println(F("  t        full auto sequence"));
  Serial.println(F("  w / s    forward / reverse      a / d   spin CCW / CW"));
  Serial.println(F("  q / e    arc left / arc right"));
  Serial.println(F("  x        stop       + / -   speed +/-10%"));
  Serial.println(F("  l        latch (disable 2.5 s dead-man)"));
  Serial.println(F("  ?        help"));
  Serial.printf (  "  speed=%d%%  minDuty=%d  maxDuty=%d  freq=%dHz  latch=%s\n",
                   speedPct, minDuty, maxDuty, FREQ_TABLE[freqIdx],
                   latchMode ? "ON" : "off");
  Serial.println(F("----------------------------------------------------\n"));
}

/* ------------------------------------------------------------------ SETUP */
void setup() {
  const int dirPins[4] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};
  for (int i = 0; i < 4; i++) { pinMode(dirPins[i], OUTPUT); digitalWrite(dirPins[i], LOW); }
  pinMode(PIN_ENA, OUTPUT); digitalWrite(PIN_ENA, LOW);
  pinMode(PIN_ENB, OUTPUT); digitalWrite(PIN_ENB, LOW);

  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n\nESP32-B ground drive v3 | HW-095 / L298N | 4x TT 1:48"));
  Serial.printf("Pins: ENA=%d IN1=%d IN2=%d IN3=%d IN4=%d ENB=%d\n",
                PIN_ENA, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENB);
  bootPinReport();
  pwmAttach();
  Serial.printf("PWM: %d Hz, %d-bit, minDuty=%d maxDuty=%d\n",
                FREQ_TABLE[freqIdx], PWM_BITS, minDuty, maxDuty);
  printHelp();
  lastCmdMs = lastRampMs = millis();
}

/* ------------------------------------------------------------------- LOOP */
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r' && c != ' ') {
      switch (c) {
        case 'o': dcBypass();                                             break;
        case 'b': breakawaySweep();                                       break;
        case '[': rawDuty = max(0,   rawDuty - 10); rawApply();           break;
        case ']': rawDuty = min(255, rawDuty + 10); rawApply();           break;
        case 'f': cycleFreq();                                            break;
        case 'w': drive( speedPct,  speedPct); Serial.println("FWD");      break;
        case 's': drive(-speedPct, -speedPct); Serial.println("REV");      break;
        case 'a': drive(-speedPct,  speedPct); Serial.println("SPIN CCW"); break;
        case 'd': drive( speedPct, -speedPct); Serial.println("SPIN CW");  break;
        case 'q': drive(speedPct / 3, speedPct); Serial.println("ARC L");  break;
        case 'e': drive(speedPct, speedPct / 3); Serial.println("ARC R");  break;
        case '1': drive( speedPct, 0); Serial.println("LEFT only");        break;
        case '2': drive( 0, speedPct); Serial.println("RIGHT only");       break;
        case 't': autoSequence();                                          break;
        case 'p': pinByPinTest();                                          break;
        case 'x': stopAll(); Serial.println("STOP");                       break;
        case '+': speedPct = min(100, speedPct + 10);
                  Serial.printf("speed %d%%\n", speedPct);                 break;
        case '-': speedPct = max(10,  speedPct - 10);
                  Serial.printf("speed %d%%\n", speedPct);                 break;
        case 'l': latchMode = !latchMode;
                  Serial.printf("latch %s\n", latchMode ? "ON" : "off");   break;
        case '?': printHelp();                                             break;
        default : Serial.printf("unknown '%c' (? for help)\n", c);         break;
      }
      lastCmdMs = millis();
    }
  }

  if (!latchMode && !rawMode &&
      (targetSigned[LEFT] || targetSigned[RIGHT]) &&
      (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    stopAll();
    Serial.println(F("[dead-man timeout] stopped"));
  }

  serviceRamp();
}
