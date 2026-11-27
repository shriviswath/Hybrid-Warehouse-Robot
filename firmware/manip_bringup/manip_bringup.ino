/* ============================================================================
 *  Hybrid Ground-Aerial Warehouse Robot  --  MANIPULATOR SERVO BRING-UP
 * ----------------------------------------------------------------------------
 *  Board   : ESP32-B  (drive / arm / radio MCU)
 *  Servos  : 2 x MG995
 *              S1  rack-and-pinion lift   -> GPIO16
 *              S2  gripper                -> GPIO4
 *  Method  : raw LEDC at 50 Hz. No ESP32Servo library -- same approach as
 *            hybridbot_app.ino so the two agree on pulse timing.
 *
 * ============================================================================
 *  !! READ BEFORE POWERING UP !!
 *
 *  1. DO NOT POWER THE SERVOS FROM THE ESP32.
 *     An MG995 draws ~0.7 A running and 1.2-2.5 A stalled. Two of them will
 *     brown out any ESP32 board's regulator instantly and can damage it.
 *     Use a separate 5-6 V supply or BEC rated >= 3 A.
 *
 *  2. COMMON GROUND. Servo supply GND <-> ESP32 GND. Mandatory.
 *     Signal wire goes to the GPIO; the servo's red wire NEVER does.
 *
 *  3. A rack-and-pinion has HARD END STOPS. Commanding the servo past them
 *     stalls it at full current until the gears strip or the coil cooks.
 *     That is why this sketch starts with NARROW soft limits (see below) and
 *     makes you widen them by hand with [ and ], one notch at a time,
 *     listening for the stall buzz.
 *
 *  4. First run: arm unloaded, gripper empty, nothing in the rack's path.
 *     Keep a hand on the supply switch.
 *
 *  5. 'd' detaches both servos (stops the pulse train) -- use it the moment
 *     anything buzzes or binds. A stalled MG995 has seconds, not minutes.
 *
 *  6. !! THE LEFT DRIVE MOTOR RUNS DURING UPLOAD !!
 *     GPIO5 (ENA) is a strapping pin that pulses at boot, and GPIO14 (IN2)
 *     emits a PWM burst from the ROM bootloader. Left enable goes active while
 *     a left direction pin pulses => the left wheels spin. The right side has
 *     no boot-active pins, which is why only the left side does it.
 *     setup() below parks all six drive pins LOW as its very first action, so
 *     it stops as soon as user code runs -- but the ROM pulse happens BEFORE
 *     any user code, and during an upload no user code runs at all.
 *     THE REAL FIX IS HARDWARE: 10k pulldowns from ENA, IN1, IN2, IN3, IN4 to
 *     GND at the HW-095 end. The L298N has no internal pulldowns.
 *
 *  Serial monitor: 115200 baud, line ending = "Newline".
 *  THIS SKETCH IS LINE-BUFFERED: type a whole command and press Enter.
 * ========================================================================= */

#include <Arduino.h>

/* ------------------------------------------------------------------ PINS */
static const int PIN_RACK = 2;      // GPIO2: also a strapping pin (download
                                    // mode) and the onboard LED on most DevKits
static const int PIN_GRIP = 4;

/* ------------------------------------------- GROUND DRIVE PINS (PARK ONLY)
 * This sketch does not drive the wheels. It only forces these LOW in setup()
 * so a boot twitch does not leave a motor running while you test the arm.
 * Keep in step with the drive sketch: ENA=D5, IN1, IN2, IN3, IN4, ENB.
 * ---------------------------------------------------------------------- */
static const int DRIVE_PINS[] = { 5, 12, 14, 27, 26, 25 };
static const int DRIVE_N = sizeof(DRIVE_PINS) / sizeof(DRIVE_PINS[0]);

/* -------------------------------------------------------------- PWM / US */
static const uint32_t SERVO_HZ   = 50;
static const uint8_t  PWM_BITS   = 16;          // 16-bit at 50 Hz = ~0.3 us steps
static const uint32_t FULL_SCALE = (1UL << PWM_BITS) - 1;

// MG995 pulse envelope.
// 500/2500 BECAUSE THAT IS WHAT THE TRAVEL LIMITS BELOW WERE MEASURED WITH.
// 600/2400 is gentler on the servo's internal stops, but changing it shifts
// every angle by ~8 deg -- INTO the end stop. If you move to 600/2400, you
// must re-measure the limits. Do not change one without the other.
static int usMin = 500;
static int usMax = 2500;

/* ---------------------------------------- MEASURED TRAVEL LIMITS ---------
 * Bench-calibrated. These are the angles just INSIDE where each mechanism
 * bottoms out. Nothing in this sketch will command outside them.
 *
 *   RACK  40..180 deg,  INVERTED SENSE:  180 = RETRACTED (parked)
 *                                         40 = EXTENDED
 *   GRIP   0..60  deg:                     0 = OPEN
 *                                         60 = CLOSED
 * ---------------------------------------------------------------------- */
static const int RACK_EXTENDED  = 40;
static const int RACK_RETRACTED = 180;
static const int GRIP_OPEN      = 0;
static const int GRIP_CLOSED    = 60;

int rackMin = RACK_EXTENDED, rackMax = RACK_RETRACTED, rackHome = RACK_RETRACTED;
int gripMin = GRIP_OPEN,     gripMax = GRIP_CLOSED,    gripHome = GRIP_OPEN;

int gripOpenDeg  = GRIP_OPEN;
int gripShutDeg  = GRIP_CLOSED;

/* ------------------------------------------------------------- MOVEMENT --
 * Servos are commanded through a slew limiter. Two MG995 snapping from one
 * end to the other together is the worst case for the supply; ramping keeps
 * the current draw spread out and stops the brown-out reset loop.
 * ---------------------------------------------------------------------- */
static int   slewDegPerSec = 90;
static const uint32_t TICK_MS = 20;             // 50 Hz update

struct Servo {
  int     pin;
  const char *name;
  float   cur;          // actual commanded angle
  int     tgt;          // where we want to be
  int     lo, hi, home;
  bool    attached;
};

Servo S[2] = {
  { PIN_RACK, "rack", (float)RACK_RETRACTED, RACK_RETRACTED,
    RACK_EXTENDED, RACK_RETRACTED, RACK_RETRACTED, false },
  { PIN_GRIP, "grip", (float)GRIP_OPEN, GRIP_OPEN,
    GRIP_OPEN, GRIP_CLOSED, GRIP_OPEN, false },
};

int sel = 0;                                     // which servo [ and ] nudge

/* ------------------------------------------------------------ LEDC SHIMS */
static void servoAttach(Servo &s) {
  if (s.attached) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(s.pin, SERVO_HZ, PWM_BITS);
#else
  int ch = (&s == &S[0]) ? 2 : 3;                // ch2/ch3 share timer1
  ledcSetup(ch, SERVO_HZ, PWM_BITS);
  ledcAttachPin(s.pin, ch);
#endif
  s.attached = true;
}

static void servoDetach(Servo &s) {
  if (!s.attached) return;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(s.pin);
#else
  ledcDetachPin(s.pin);
#endif
  pinMode(s.pin, INPUT);                         // high-Z: servo goes limp
  s.attached = false;
}

static void servoWriteUs(Servo &s, int us) {
  if (!s.attached) return;
  uint32_t duty = (uint32_t)((uint64_t)us * FULL_SCALE * SERVO_HZ / 1000000ULL);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(s.pin, duty);
#else
  ledcWrite((&s == &S[0]) ? 2 : 3, duty);
#endif
}

static int degToUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return usMin + (int)((usMax - usMin) * deg / 180.0f);
}

/* ---------------------------------------------------------------- MOTION */
static void setTarget(int i, int deg) {
  Servo &s = S[i];
  int clamped = constrain(deg, s.lo, s.hi);
  if (clamped != deg)
    Serial.printf("  !! %d deg is outside the %s soft limits [%d..%d] -> clamped to %d\n",
                  deg, s.name, s.lo, s.hi, clamped);
  servoAttach(s);
  s.tgt = clamped;
}

static void serviceSlew() {
  static uint32_t last = 0;
  if (millis() - last < TICK_MS) return;
  last = millis();
  float step = (float)slewDegPerSec * TICK_MS / 1000.0f;

  for (int i = 0; i < 2; i++) {
    Servo &s = S[i];
    if (!s.attached) continue;
    if (fabs(s.tgt - s.cur) < 0.05f) { s.cur = s.tgt; }
    else if (s.cur < s.tgt) s.cur = min((float)s.tgt, s.cur + step);
    else                    s.cur = max((float)s.tgt, s.cur - step);
    servoWriteUs(s, degToUs(s.cur));
  }
}

static void waitSettled(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { serviceSlew(); delay(2); }
}

/* ------------------------------------------------- GROUND DRIVE PARKING --
 * Forces every L298N control line LOW. Called first thing in setup(), and
 * again from the 'drv' command if you ever suspect a stray pin.
 * ---------------------------------------------------------------------- */
static void parkDrivePins() {
  for (int i = 0; i < DRIVE_N; i++) {
    pinMode(DRIVE_PINS[i], OUTPUT);
    digitalWrite(DRIVE_PINS[i], LOW);
  }
}

/* --------------------------------------------------------------- ACTIONS */
static void nudge(int d) {
  Servo &s = S[sel];
  int want = s.tgt + d;
  if (want < s.lo || want > s.hi) {
    Serial.printf("  %s is at a soft limit (%d). Widen it with 'lim' if the\n"
                  "  mechanism really does travel further -- 2 deg at a time,\n"
                  "  and stop the instant you hear the servo buzz.\n",
                  s.name, s.tgt);
    return;
  }
  setTarget(sel, want);
  Serial.printf("  %s -> %d deg  (%d us)\n", s.name, s.tgt, degToUs((float)s.tgt));
}

static void centerAll() {
  Serial.println(F("  centering both servos"));
  for (int i = 0; i < 2; i++) { servoAttach(S[i]); setTarget(i, S[i].home); }
}

static void detachAll() {
  for (int i = 0; i < 2; i++) servoDetach(S[i]);
  Serial.println(F("  both servos DETACHED (limp, no holding torque)"));
}

/* Safe parked pose: release the load FIRST, then pull the rack in. */
static void parkPose() {
  Serial.println(F("\n  parking: gripper open, then rack retracted"));
  setTarget(1, GRIP_OPEN);      waitSettled(1200);
  setTarget(0, RACK_RETRACTED); waitSettled(2000);
  Serial.println(F("  parked.\n"));
}

/* Declare where a servo ACTUALLY is, without moving it. Needed because the
 * firmware cannot read a shaft position -- it only knows what it last sent.
 * Detach first, or the correction becomes a move. */
static void declarePos(int i, int deg) {
  Servo &s = S[i];
  if (s.attached) { Serial.println(F("  detach first ('d'), or it will jump")); return; }
  s.cur = s.tgt = constrain(deg, s.lo, s.hi);
  Serial.printf("  %s is now assumed to be at %d deg (nothing moved)\n", s.name, s.tgt);
}

static void sweepOne(int i) {
  Servo &s = S[i];
  Serial.printf("\n=== SWEEP %s : %d -> %d -> %d deg, slowly ===\n",
                s.name, s.lo, s.hi, s.home);
  Serial.println(F("  Listen for buzzing or grinding. Press any key to abort.\n"));
  servoAttach(s);
  int seq[3] = {s.lo, s.hi, s.home};
  for (int k = 0; k < 3; k++) {
    setTarget(i, seq[k]);
    Serial.printf("  -> %3d deg\n", seq[k]);
    uint32_t t0 = millis();
    while (millis() - t0 < 1400) {
      serviceSlew();
      if (Serial.available()) { Serial.read(); detachAll();
        Serial.println(F("  ABORTED, servos detached.")); return; }
      delay(2);
    }
  }
  Serial.println(F("=== SWEEP DONE ===\n"));
}

static void gripCycle() {
  Serial.println(F("\n=== GRIPPER OPEN / CLOSE CYCLE x3 ==="));
  for (int k = 0; k < 3; k++) {
    Serial.printf("  open  (%d deg)\n", gripOpenDeg); setTarget(1, gripOpenDeg); waitSettled(1200);
    Serial.printf("  close (%d deg)\n", gripShutDeg); setTarget(1, gripShutDeg); waitSettled(1200);
  }
  Serial.println(F("=== CYCLE DONE ===\n"));
}

static void pickPlaceDemo() {
  Serial.println(F("\n=== PICK-AND-PLACE MOTION DEMO (no cargo) ==="));
  setTarget(1, GRIP_OPEN);      Serial.println(F("  1. gripper open"));      waitSettled(1000);
  setTarget(0, RACK_EXTENDED);  Serial.println(F("  2. rack EXTEND  (40)")); waitSettled(2000);
  setTarget(1, GRIP_CLOSED);    Serial.println(F("  3. gripper close"));     waitSettled(1200);
  setTarget(0, RACK_RETRACTED); Serial.println(F("  4. rack RETRACT (180)"));waitSettled(2000);
  waitSettled(800);
  setTarget(0, RACK_EXTENDED);  Serial.println(F("  5. rack EXTEND  (40)")); waitSettled(2000);
  setTarget(1, GRIP_OPEN);      Serial.println(F("  6. gripper open"));      waitSettled(1200);
  setTarget(0, RACK_RETRACTED); Serial.println(F("  7. rack RETRACT (180)"));waitSettled(2000);
  Serial.println(F("=== DEMO DONE ===\n"));
}

static void showState() {
  Serial.println(F("\n  servo  pin   now   tgt   limits      home   attached"));
  for (int i = 0; i < 2; i++) {
    Serial.printf("  %-5s  %-4d  %4.0f  %4d   [%3d..%3d]   %3d    %s\n",
                  S[i].name, S[i].pin, S[i].cur, S[i].tgt,
                  S[i].lo, S[i].hi, S[i].home, S[i].attached ? "yes" : "no");
  }
  Serial.printf("  pulse envelope %d..%d us   slew %d deg/s   selected: %s\n",
                usMin, usMax, slewDegPerSec, S[sel].name);
  Serial.printf("  grip presets: open=%d  shut=%d\n\n", gripOpenDeg, gripShutDeg);
}

static void printHelp() {
  Serial.println(F("\n--------------- MANIPULATOR SERVO TEST ---------------"));
  Serial.println(F("  TYPE A COMMAND AND PRESS ENTER (line-buffered)"));
  Serial.println(F("  r <deg>        move rack to angle"));
  Serial.println(F("  g <deg>        move gripper to angle"));
  Serial.println(F("  sel r | sel g  choose which servo [ and ] nudge"));
  Serial.println(F("  [ | ]          nudge selected servo -2 / +2 deg"));
  Serial.println(F("  c              centre both to home"));
  Serial.println(F("  d              DETACH both (limp) -- use if anything buzzes"));
  Serial.println(F("  sweep r | g    slow lo -> hi -> home sweep, any key aborts"));
  Serial.println(F("  cycle          gripper open/close x3"));
  Serial.println(F("  demo           full pick-and-place motion, no cargo"));
  Serial.println(F("  ext | ret      rack extended (40) / retracted (180)"));
  Serial.println(F("  op  | cl       gripper open (0) / closed (60)"));
  Serial.println(F("  park           gripper open, THEN rack retracted"));
  Serial.println(F("  set r 180      declare real angle, no movement (detach first)"));
  Serial.println(F("  drv            re-park the ground drive pins LOW"));
  Serial.println(F("  lim r 60 130   set soft limits for a servo"));
  Serial.println(F("  home r 90      set the home angle"));
  Serial.println(F("  open <deg>     set the gripper OPEN preset"));
  Serial.println(F("  shut <deg>     set the gripper CLOSED preset"));
  Serial.println(F("  us 600 2400    set the pulse envelope in microseconds"));
  Serial.println(F("  slew <deg/s>   movement speed, default 90"));
  Serial.println(F("  st             show state"));
  Serial.println(F("  ?              this help"));
  Serial.println(F("------------------------------------------------------"));
  showState();
}

/* -------------------------------------------------------------- PARSER -- */
static int servoByLetter(char c) { return (c == 'r') ? 0 : (c == 'g') ? 1 : -1; }

static void handleLine(char *line) {
  char verb[12] = {0};
  int a = 0, b = 0;
  char which = 0;

  if (sscanf(line, "%11s", verb) != 1) return;

  if (!strcmp(verb, "?") || !strcmp(verb, "help")) { printHelp(); return; }
  if (!strcmp(verb, "st"))   { showState();  return; }
  if (!strcmp(verb, "c"))    { centerAll();  return; }
  if (!strcmp(verb, "d"))    { detachAll();  return; }
  if (!strcmp(verb, "["))    { nudge(-2);    return; }
  if (!strcmp(verb, "]"))    { nudge(+2);    return; }
  if (!strcmp(verb, "cycle")){ gripCycle();  return; }
  if (!strcmp(verb, "demo")) { pickPlaceDemo(); return; }
  if (!strcmp(verb, "park")) { parkPose();     return; }
  if (!strcmp(verb, "ext"))  { setTarget(0, RACK_EXTENDED);  return; }
  if (!strcmp(verb, "ret"))  { setTarget(0, RACK_RETRACTED); return; }
  if (!strcmp(verb, "op"))   { setTarget(1, GRIP_OPEN);      return; }
  if (!strcmp(verb, "cl"))   { setTarget(1, GRIP_CLOSED);    return; }
  if (!strcmp(verb, "drv"))  { parkDrivePins();
    Serial.println(F("  ground drive pins forced LOW")); return; }

  if (!strcmp(verb, "set")) {
    if (sscanf(line, "%*s %c %d", &which, &a) == 2 && servoByLetter(which) >= 0)
      declarePos(servoByLetter(which), a);
    else Serial.println(F("  usage: set r 180   /   set g 0"));
    return;
  }

  if (!strcmp(verb, "sel")) {
    if (sscanf(line, "%*s %c", &which) == 1 && servoByLetter(which) >= 0) {
      sel = servoByLetter(which);
      Serial.printf("  selected: %s\n", S[sel].name);
    } else Serial.println(F("  usage: sel r | sel g"));
    return;
  }
  if (!strcmp(verb, "sweep")) {
    if (sscanf(line, "%*s %c", &which) == 1 && servoByLetter(which) >= 0)
      sweepOne(servoByLetter(which));
    else Serial.println(F("  usage: sweep r | sweep g"));
    return;
  }
  if (!strcmp(verb, "lim")) {
    if (sscanf(line, "%*s %c %d %d", &which, &a, &b) == 3 && servoByLetter(which) >= 0) {
      int i = servoByLetter(which);
      if (a < 0 || b > 180 || a >= b) { Serial.println(F("  bad range")); return; }
      S[i].lo = a; S[i].hi = b;
      S[i].home = constrain(S[i].home, a, b);
      Serial.printf("  %s limits now [%d..%d]\n", S[i].name, a, b);
    } else Serial.println(F("  usage: lim r 60 130"));
    return;
  }
  if (!strcmp(verb, "home")) {
    if (sscanf(line, "%*s %c %d", &which, &a) == 2 && servoByLetter(which) >= 0) {
      int i = servoByLetter(which);
      S[i].home = constrain(a, S[i].lo, S[i].hi);
      Serial.printf("  %s home now %d\n", S[i].name, S[i].home);
    } else Serial.println(F("  usage: home r 90"));
    return;
  }
  if (!strcmp(verb, "open") || !strcmp(verb, "shut")) {
    if (sscanf(line, "%*s %d", &a) == 1) {
      a = constrain(a, S[1].lo, S[1].hi);
      if (verb[0] == 'o') { gripOpenDeg = a; Serial.printf("  grip open  = %d\n", a); }
      else                { gripShutDeg = a; Serial.printf("  grip shut  = %d\n", a); }
    } else Serial.println(F("  usage: open 70   /   shut 110"));
    return;
  }
  if (!strcmp(verb, "us")) {
    if (sscanf(line, "%*s %d %d", &a, &b) == 2 && a >= 400 && b <= 2600 && a < b) {
      usMin = a; usMax = b;
      Serial.printf("  pulse envelope now %d..%d us\n", usMin, usMax);
    } else Serial.println(F("  usage: us 600 2400   (400..2600, min < max)"));
    return;
  }
  if (!strcmp(verb, "slew")) {
    if (sscanf(line, "%*s %d", &a) == 1 && a >= 5 && a <= 400) {
      slewDegPerSec = a; Serial.printf("  slew = %d deg/s\n", a);
    } else Serial.println(F("  usage: slew 90   (5..400)"));
    return;
  }
  if (!strcmp(verb, "r") || !strcmp(verb, "g")) {
    int i = servoByLetter(verb[0]);
    if (sscanf(line, "%*s %d", &a) == 1) {
      setTarget(i, a);
      Serial.printf("  %s -> %d deg  (%d us)\n", S[i].name, S[i].tgt, degToUs((float)S[i].tgt));
    } else { sel = i; Serial.printf("  selected: %s\n", S[i].name); }
    return;
  }

  Serial.printf("  unknown command '%s'  (? for help)\n", verb);
}

/* ------------------------------------------------------------------ SETUP */
void setup() {
  // FIRST ACTION, before anything else: kill any drive output left running by
  // the boot pulse on GPIO5 / GPIO14. Costs microseconds, saves a gearbox.
  parkDrivePins();

  Serial.begin(115200);
  delay(400);
  Serial.println(F("\n\nESP32-B manipulator servo bring-up  |  2x MG995"));
  Serial.printf("Pins: RACK=GPIO%d  GRIP=GPIO%d\n", PIN_RACK, PIN_GRIP);
  Serial.println(F("Ground drive pins parked LOW (ENA/IN1..IN4/ENB)."));
  Serial.println(F("  If a wheel still spins during UPLOAD, that is the ROM"));
  Serial.println(F("  bootloader pulsing GPIO5/GPIO14 before any code runs."));
  Serial.println(F("  Fit 10k pulldowns on the L298N inputs -- firmware cannot"));
  Serial.println(F("  prevent it.\n"));
  Serial.println(F("Servos start DETACHED, assumed at rack=180, grip=0."));
  Serial.println(F("  If that is not where they really are: set r <deg>, set g <deg>"));
  Serial.println(F("  then 'c' to take up position slowly."));
  Serial.println(F("Separate 5-6 V servo supply with common ground, or stop now."));
  printHelp();
}

/* ------------------------------------------------------------------- LOOP */
void loop() {
  static char buf[48];
  static uint8_t n = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { buf[n] = 0; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
  serviceSlew();
}
