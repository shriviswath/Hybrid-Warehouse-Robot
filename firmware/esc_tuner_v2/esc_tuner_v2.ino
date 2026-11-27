/* ============================================================================
 *  ESC TUNER v2  --  ESP32-A, four ESCs
 * ----------------------------------------------------------------------------
 *  >>> PROPELLERS OFF. FRAME CLAMPED OR HELD DOWN. <<<
 *
 *  An ESC reads a pulse repeated 50 times a second. The pulse WIDTH is the
 *  throttle: ~1000 us = stop, ~2000 us = full. Frequency never changes.
 *
 *  ESC1 = GPIO23   ESC2 = GPIO22   ESC3 = GPIO21   ESC4 = GPIO19
 *  GPIO34-39 are input-only -- an ESC wired there can never get a signal.
 *
 * ----------------------------------------------------------------------------
 *  WHAT v2 ADDS, AND WHY
 *
 *  ARMING          Throttle above minimum is refused until you type 'arm'.
 *                  You must be at minimum to arm. A mistyped 1800 on a
 *                  disarmed tuner does nothing at all.
 *
 *  THROTTLE CAP    Default ceiling 1300 us. Everything above it is clamped
 *                  and reported. Raise deliberately with 'cap 1600'. You
 *                  will not reach full throttle by accident.
 *
 *  DEAD-MAN        8 s with no serial input while spinning -> disarm, all
 *                  motors to 1000. ESCs hold their last command forever;
 *                  a crashed terminal or an unplugged USB otherwise leaves
 *                  four motors running.
 *
 *  SLEW            Throttle moves at a limited us/s instead of stepping.
 *                  Protects the frame, the mounts, and the pack from a
 *                  torque step. 'jump' disables it for one command.
 *
 *  MOTOR ID        't' bumps each motor in turn at a low throttle so you can
 *                  map ESC1..4 to physical arms and record spin direction.
 *                  Do this BEFORE any flight-controller frame setup.
 *
 *  BREAKAWAY LOG   During 's', press 'k' when the selected motor first turns.
 *                  The value is stored per motor and shown in 'st'.
 *
 *  BROWNOUT        Reset reason printed at boot. If it says BROWNOUT, the
 *                  pack sagged -- that is a power fault, not a code fault.
 *
 *  Serial: 115200. Any line ending; a 300 ms pause also counts as Enter.
 * ========================================================================= */

#include <Arduino.h>
#include <esp_system.h>

/* ------------------------------------------------------------------ PINS --
 * MOTOR ORDER, not pin order. Index 0 is ArduPilot motor 1.
 *
 * Remapped 18 Nov 2026 to match the physical quad layout:
 *     M1 -> GPIO19
 *     M2 -> GPIO22
 *     M3 -> GPIO23
 *     M4 -> GPIO21
 *
 * The SAME order must go into the ArduPilot board header's RCOUT list, or
 * the bench numbering and the flight numbering disagree and the quad flips
 * on takeoff. Verify with 't' before trusting it.
 * ---------------------------------------------------------------------- */
static const int ESC_PINS[4] = {19, 22, 23, 21};   // M1, M2, M3, M4
static const int ESC_CH[4]   = {4, 5, 6, 7};        // ch4/5 -> timer2, ch6/7 -> timer3

static const int US_MIN = 1000, US_MAX = 2000;

/* --------------------------------------------------------------- LIMITS */
int  throttleCap   = 1300;          // bench ceiling. 'cap <us>' to change.
int  slewUsPerSec  = 300;           // 0 = instant
static const uint32_t DEADMAN_MS = 8000;
static const uint32_t TICK_MS    = 20;              // 50 Hz service

/* ---------------------------------------------------------------- STATE */
bool  sel[4]     = {true, false, false, false};
int   tgtUs[4]   = {US_MIN, US_MIN, US_MIN, US_MIN};
float curUs[4]   = {US_MIN, US_MIN, US_MIN, US_MIN};
int   firstSpin[4] = {0, 0, 0, 0};                  // breakaway, 0 = unknown
bool  armed      = false;
bool  nextJump   = false;
uint32_t lastInputMs = 0;

/* ------------------------------------------------------------------ LEDC */
static uint32_t usToDuty(int us) { return (uint32_t)(((uint32_t)us * 65535UL) / 20000UL); }

static void escAttach() {
  for (int i = 0; i < 4; i++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(ESC_PINS[i], 50, 16, ESC_CH[i]);
#else
    ledcSetup(ESC_CH[i], 50, 16);
    ledcAttachPin(ESC_PINS[i], ESC_CH[i]);
#endif
  }
}

static void escRaw(int i, int us) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ESC_PINS[i], usToDuty(us));
#else
  ledcWrite(ESC_CH[i], usToDuty(us));
#endif
}

/* -------------------------------------------------------------- CONTROL */
static bool anySpinning() {
  for (int i = 0; i < 4; i++) if (tgtUs[i] > US_MIN || curUs[i] > US_MIN + 1) return true;
  return false;
}

static void allStop(bool quiet = false) {
  for (int i = 0; i < 4; i++) { tgtUs[i] = US_MIN; curUs[i] = US_MIN; escRaw(i, US_MIN); }
  nextJump = false;
  if (!quiet) Serial.println(F("  all motors -> 1000 us"));
}

static void disarm(const char *why) {
  allStop(true);
  if (armed) Serial.printf("  DISARMED (%s). All motors at 1000 us.\n", why);
  armed = false;
}

static void setSelected(int us) {
  if (us > throttleCap) {
    Serial.printf("  %d us exceeds the cap of %d -> clamped. 'cap %d' to raise it.\n",
                  us, throttleCap, us);
    us = throttleCap;
  }
  if (!armed && us > US_MIN) {
    Serial.println(F("  REFUSED: not armed. Type 'arm' first (props off!)."));
    return;
  }
  for (int i = 0; i < 4; i++) if (sel[i]) tgtUs[i] = constrain(us, US_MIN, US_MAX);
}

// Called at 50 Hz. Slew-limits every channel toward its target.
static void serviceThrottle() {
  static uint32_t last = 0;
  if (millis() - last < TICK_MS) return;
  last = millis();

  float step = nextJump || slewUsPerSec <= 0
                 ? 10000.0f
                 : (float)slewUsPerSec * TICK_MS / 1000.0f;

  bool settled = true;
  for (int i = 0; i < 4; i++) {
    if (fabs(tgtUs[i] - curUs[i]) < 0.5f)      curUs[i] = tgtUs[i];
    else if (curUs[i] < tgtUs[i]) { curUs[i] = min((float)tgtUs[i], curUs[i] + step); settled = false; }
    else                          { curUs[i] = max((float)tgtUs[i], curUs[i] - step); settled = false; }
    escRaw(i, (int)curUs[i]);
  }
  if (settled) nextJump = false;
}

/* ----------------------------------------------------------------- UTIL */
static void flushIn() { while (Serial.available()) Serial.read(); }

// Waits while keeping the throttle service running. Returns the key pressed,
// or 0 on timeout. 'x' is handled here so stop always works instantly.
static char waitKey(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    serviceThrottle();
    if (Serial.available()) {
      char c = Serial.read();
      lastInputMs = millis();
      delay(20); flushIn();
      if (c == 'x') { disarm("stop key"); return 'x'; }
      return c;
    }
    delay(2);
  }
  return 0;
}

static void printState() {
  Serial.printf("\n  %s   cap %d us   slew %d us/s\n",
                armed ? "** ARMED **" : "disarmed", throttleCap, slewUsPerSec);
  Serial.println(F("  mot  gpio  sel  target  actual  first-spin"));
  for (int i = 0; i < 4; i++)
    Serial.printf("   %d   %-4d  %-3s  %5d   %5.0f   %s%d\n",
                  i + 1, ESC_PINS[i], sel[i] ? "<<" : "  ",
                  tgtUs[i], curUs[i],
                  firstSpin[i] ? "" : "unknown ", firstSpin[i]);
  Serial.println();
}

/* ---------------------------------------------------------------- SWEEP */
static void sweep() {
  if (!armed) { Serial.println(F("  arm first ('arm'), props off.")); return; }
  Serial.println(F("\n=== SWEEP on the selection. 'k' marks first spin, any other key stops. ==="));
  flushIn();
  setSelected(US_MIN);
  for (int us = US_MIN; us <= throttleCap; us += 25) {
    setSelected(us);
    Serial.printf("  pulse = %d us  (%d%% of 1000-2000)\n", us, (us - US_MIN) / 10);
    char k = waitKey(400);
    if (k == 'k') {
      for (int i = 0; i < 4; i++) if (sel[i]) firstSpin[i] = us;
      Serial.printf("  >>> first spin recorded at %d us\n", us);
    } else if (k) {
      allStop();
      Serial.printf("  STOPPED at %d us\n", us);
      return;
    }
  }
  Serial.printf("  reached the cap (%d us), holding 1 s\n", throttleCap);
  waitKey(1000);
  allStop();
  Serial.println(F("=== SWEEP DONE ==="));
}

/* ------------------------------------------------------------- MOTOR ID */
static void motorIdTest() {
  if (!armed) { Serial.println(F("  arm first ('arm'), props off.")); return; }
  int us = firstSpin[0] ? min(firstSpin[0] + 40, throttleCap) : min(1150, throttleCap);
  Serial.println(F("\n=== MOTOR IDENTIFICATION ==="));
  Serial.printf("  Each motor runs alone for 2 s at %d us.\n", us);
  Serial.println(F("  WRITE DOWN for each: which arm it is, and which way it turns."));
  Serial.println(F("  You need this before any flight-controller frame setup."));
  Serial.println(F("  Any key aborts.\n"));
  bool save[4]; for (int i = 0; i < 4; i++) save[i] = sel[i];

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) sel[j] = (j == i);
    Serial.printf("  --> M%d  (GPIO%d)  running...\n", i + 1, ESC_PINS[i]);
    setSelected(us);
    if (waitKey(2000)) { allStop(); Serial.println(F("  ABORTED")); break; }
    setSelected(US_MIN);
    if (waitKey(900))  { allStop(); Serial.println(F("  ABORTED")); break; }
  }
  for (int i = 0; i < 4; i++) sel[i] = save[i];
  allStop();
  Serial.println(F("=== ID TEST DONE ===\n"));
}

/* --------------------------------------------------------- CALIBRATION --
 * Teaches the ESC what minimum and maximum mean. An uncalibrated ESC may not
 * arm at 1000 us, or may start at a different value from its neighbours --
 * the usual reason some motors spin in a test and others do not.
 * ---------------------------------------------------------------------- */
static void calibrate() {
  Serial.println(F("\n=== THROTTLE CALIBRATION for the selection ==="));
  Serial.println(F("  PROPS OFF. This sends FULL throttle, cap and arming bypassed."));
  Serial.println(F("  1. DISCONNECT the battery now."));
  Serial.println(F("  2. Press any key -> 2000 us will be sent."));
  flushIn();
  while (!Serial.available()) delay(10);
  delay(30); flushIn();

  for (int i = 0; i < 4; i++) if (sel[i]) { tgtUs[i] = curUs[i] = US_MAX; escRaw(i, US_MAX); }
  Serial.println(F("  -> 2000 us."));
  Serial.println(F("  3. CONNECT the battery. Wait for the rising tune, then beeps."));
  Serial.println(F("  4. Press any key -> 1000 us."));
  while (!Serial.available()) delay(10);
  delay(30); flushIn();

  for (int i = 0; i < 4; i++) if (sel[i]) { tgtUs[i] = curUs[i] = US_MIN; escRaw(i, US_MIN); }
  Serial.println(F("  -> 1000 us. The ESC should give its confirm beeps."));
  armed = false;
  Serial.println(F("=== CALIBRATION DONE. 'arm', then try 1150. ===\n"));
}

/* ----------------------------------------------------------------- HELP */
static void help() {
  Serial.println(F("\n---------------- ESC TUNER v2 ----------------"));
  Serial.println(F("  arm | disarm   enable / disable throttle above 1000"));
  Serial.println(F("  m1 m2 m3 m4    select one MOTOR (ArduPilot numbering)"));
  Serial.println(F("                 ma   select all"));
  Serial.println(F("  1000..2000     set pulse width (clamped to the cap)"));
  Serial.println(F("  cap <us>       throttle ceiling, 1000..2000"));
  Serial.println(F("  slew <us/s>    ramp rate, 0 = instant"));
  Serial.println(F("  jump           next change ignores slew (one shot)"));
  Serial.println(F("  s              sweep to cap; 'k' marks first spin"));
  Serial.println(F("  t              motor ID: each ESC alone for 2 s"));
  Serial.println(F("  c              throttle calibration (battery off first)"));
  Serial.println(F("  x              STOP + disarm"));
  Serial.println(F("  st | ?         state / help"));
  Serial.println(F("  M1=19  M2=22  M3=23  M4=21            PROPELLERS OFF"));
  Serial.println(F("----------------------------------------------"));
  printState();
}

/* --------------------------------------------------------------- PARSER */
static void handle(const char *cmd) {
  if (!cmd[0]) return;
  lastInputMs = millis();

  if (!strcmp(cmd, "x"))      { disarm("stop"); printState(); return; }
  if (!strcmp(cmd, "arm")) {
    if (anySpinning()) { Serial.println(F("  refuse: throttle is not at minimum")); return; }
    armed = true;
    Serial.printf("  ** ARMED **  cap %d us. Props off? 'x' stops everything.\n", throttleCap);
    return;
  }
  if (!strcmp(cmd, "disarm")) { disarm("command"); return; }
  if (!strcmp(cmd, "st"))     { printState(); return; }
  if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) { help(); return; }
  if (!strcmp(cmd, "s"))      { sweep();     return; }
  if (!strcmp(cmd, "t"))      { motorIdTest(); return; }
  if (!strcmp(cmd, "c"))      { calibrate(); return; }
  if (!strcmp(cmd, "jump"))   { nextJump = true; Serial.println(F("  next change: no slew")); return; }

  if (!strncmp(cmd, "cap", 3)) {
    int v = atoi(cmd + 3);
    if (v >= US_MIN && v <= US_MAX) {
      throttleCap = v;
      Serial.printf("  cap = %d us", throttleCap);
      if (v > 1500) Serial.print(F("   <-- that is a lot of thrust on a bench"));
      Serial.println();
      for (int i = 0; i < 4; i++) if (tgtUs[i] > throttleCap) tgtUs[i] = throttleCap;
    } else Serial.println(F("  usage: cap 1300   (1000..2000)"));
    return;
  }
  if (!strncmp(cmd, "slew", 4)) {
    int v = atoi(cmd + 4);
    if (v >= 0 && v <= 5000) { slewUsPerSec = v;
      Serial.printf("  slew = %d us/s%s\n", v, v ? "" : "  (instant)"); }
    else Serial.println(F("  usage: slew 300   (0..5000)"));
    return;
  }
  if (cmd[0] == 'm' && cmd[1]) {
    if (cmd[1] == 'a') { for (int i = 0; i < 4; i++) sel[i] = true; }
    else if (cmd[1] >= '1' && cmd[1] <= '4') {
      for (int i = 0; i < 4; i++) sel[i] = false;
      sel[cmd[1] - '1'] = true;
    } else { Serial.println(F("  use m1, m2, m3, m4 or ma")); return; }
    allStop(true);                     // changing selection always stops first
    printState();
    return;
  }
  if (cmd[0] >= '0' && cmd[0] <= '9') {
    int us = atoi(cmd);
    if (us < US_MIN || us > US_MAX) { Serial.println(F("  pulse must be 1000 to 2000")); return; }
    setSelected(us);
    printState();
    return;
  }
  Serial.printf("  unknown '%s'  (? for help)\n", cmd);
}

/* ---------------------------------------------------------------- SETUP */
static const char* resetWhy(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_BROWNOUT: return "BROWNOUT -- the pack sagged, fix power first";
    case ESP_RST_PANIC:    return "PANIC -- firmware exception";
    case ESP_RST_EXT:      return "external reset";
    case ESP_RST_SW:       return "software restart";
    default:               return "other";
  }
}

void setup() {
  for (int i = 0; i < 4; i++) { pinMode(ESC_PINS[i], OUTPUT); digitalWrite(ESC_PINS[i], LOW); }
  Serial.begin(115200);
  delay(500);
  escAttach();
  allStop(true);                       // 1000 us from the first instant so ESCs can arm

  Serial.println(F("\n\n=== ESC TUNER v2 (ESP32-A) ==="));
  Serial.printf("last reset: %s\n", resetWhy(esp_reset_reason()));
  Serial.println(F("Starts DISARMED. Throttle above 1000 us is refused until 'arm'."));
  help();
  lastInputMs = millis();
}

/* ----------------------------------------------------------------- LOOP */
void loop() {
  static char buf[12];
  static int  n = 0;
  static uint32_t lastChar = 0;

  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\n' || c == '\r') { buf[n] = 0; handle(buf); n = 0; }
    else if (c != ' ' && n < 11)  buf[n++] = c;
  }
  // no line ending configured: a 300 ms pause counts as Enter
  if (n > 0 && millis() - lastChar > 300) { buf[n] = 0; handle(buf); n = 0; }

  serviceThrottle();

  // Dead-man. An ESC holds its last command forever; a dead terminal must not
  // leave four motors running.
  if (armed && anySpinning() && millis() - lastInputMs > DEADMAN_MS)
    disarm("no input for 8 s");
}
