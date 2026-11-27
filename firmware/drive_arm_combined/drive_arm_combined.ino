/* ============================================================================
 *  Hybrid Ground-Aerial Warehouse Robot
 *  COMBINED GROUND DRIVE + MANIPULATOR BRING-UP   (both live simultaneously)
 * ----------------------------------------------------------------------------
 *  Board : ESP32-B
 *  Drive : HW-095 (L298N), 4x TT 1:48 gearmotor, 2 per side in parallel
 *            ENA=D5  IN1=D12  IN2=D14  IN3=D27  IN4=D26  ENB=D25
 *  Arm   : 2x MG995
 *            RACK -> GPIO2    GRIP -> GPIO4
 *
 *  Drive ramps and servo slews are serviced every pass, including inside the
 *  scripted sequences, so you can type an arm command while the wheels are
 *  turning and vice versa. The ONE place that blocks is drvHardStop(), which
 *  holds the brake for 150 ms -- it services both ramp engines while it does,
 *  but it cannot poll the console (that would make 'x' recurse: drvHardStop
 *  is reached FROM pollSerial -> handleLine).
 *
 *  'kill' and 'x' set abortSeq, which every blocking wait and every sequence
 *  loop honours. That is what makes 'kill' an actual panic word.
 *
 * ============================================================================
 *  !! POWER -- READ THIS, IT IS THE POINT OF A COMBINED TEST !!
 *
 *  Worst-case simultaneous draw:
 *      4x TT motors stalled      ~2.2 A per side   = ~4.4 A
 *      2x MG995 stalled          ~1.2-2.5 A each   = up to 5 A
 *      -------------------------------------------------------
 *      well over 9 A if everything binds at once
 *
 *  - Servos on their OWN 5-6 V supply or BEC, >= 6 A to cover the 5 A worst
 *    case computed above with margin. NEVER off the ESP32.
 *  - Motors on the pack through the L298N.
 *  - ALL grounds tied together, including the ESP32's.
 *  - If the ESP32 reboots mid-test, the supply sagged. Lower 'slew' and
 *    'maxd' before blaming the code. A brownout reset mid-move leaves the
 *    firmware's idea of the servo angle wrong -- re-declare it with 'set'.
 *
 *  !! LEDC CHANNEL MAP -- DO NOT LET THE CORE AUTO-ALLOCATE !!
 *      ch0 / ch1  ->  timer0  ->  1 kHz,  8-bit   ->  ENA, ENB
 *      ch2 / ch3  ->  timer1  ->  50 Hz, 16-bit   ->  RACK, GRIP
 *  Channels pair onto timers two at a time. If a servo lands on timer0 it
 *  reprograms that timer to 50 Hz and your motor PWM becomes a 50 Hz chopper.
 *  Every attach below names its channel explicitly for exactly this reason.
 *
 *  !! LEFT WHEEL TWITCHES ON RESET AND DURING UPLOAD !!
 *  GPIO5 (ENA) is a strapping pin that pulses at boot; GPIO14 (IN2) emits a
 *  PWM burst from the ROM bootloader. Both happen before user code runs, so
 *  firmware cannot prevent it. FIT 10k PULLDOWNS from ENA, IN1, IN2, IN3, IN4,
 *  ENB and both servo signal lines to GND at the HW-095 end. The L298N has no
 *  internal pulldowns.
 *
 *  GPIO12 (IN1) is MTDI: held HIGH at reset it sets VDD_SDIO to 1.8 V and a
 *  3.3 V-flash module will not boot. The pin report latches driveLockout if it
 *  sees that, and drvCmd() then refuses to drive until you fix the wiring.
 *
 *  Serial: 115200, line ending = "Newline". LINE-BUFFERED: type, press Enter.
 *  First run: wheels OFF THE GROUND, arm unloaded, nothing in the rack's path.
 * ========================================================================= */

#include <Arduino.h>

/* ========================================================================
 * PINS
 * ===================================================================== */
static const int PIN_ENA = 5;    // left  enable (PWM)  -- strapping pin
static const int PIN_IN1 = 12;   // left  dir A         -- MTDI strapping pin
static const int PIN_IN2 = 14;   // left  dir B         -- boot-pulse pin
static const int PIN_IN3 = 27;   // right dir A
static const int PIN_IN4 = 26;   // right dir B
static const int PIN_ENB = 25;   // right enable (PWM)

static const int PIN_RACK = 2;   // GPIO2: strapping pin + onboard LED
static const int PIN_GRIP = 4;

/* ------------------------------------------------- explicit LEDC channels */
static const int CH_ENA  = 0;
static const int CH_ENB  = 1;
static const int CH_RACK = 2;
static const int CH_GRIP = 3;

/* ========================================================================
 * DRIVE CONFIG
 * ===================================================================== */
/* PWM FREQUENCY: 1 kHz, NOT 5 kHz.
 * The L298N is a bipolar Darlington bridge, not a MOSFET one. Turn-off storage
 * time is tens of microseconds, so at 5 kHz (200 us period) a short on-pulse is
 * mostly consumed by switching and low duties produce far less torque than the
 * number suggests. 500-1000 Hz gives the most usable low-speed torque.
 * Changeable at runtime with 'freq <hz>' -- no reflash needed. */
int  pwmFreqHz = 1000;
static const int DRV_BITS = 8;
static const int DRV_FS   = 255;

/* DUTY CLAMP -- sized to the PACK, not to taste.
 * The yellow TT gearmotor is a 3-6 V part. For sustained driving set
 *     maxDuty = 255 * 6 / (Vpack - 2)
 * 11.1 V 3S -> 168.   12.6 V 3S full -> 145.   7.4 V 2S -> 283, i.e. no clamp.
 * (The 145 default is the 12.6 V figure. Recompute it for YOUR measured pack
 * voltage -- the old comment claimed 145 came from 11.1 V, which it does not.)
 * 145 at 5 kHz was what stopped v2 moving at all. If the wheels still will
 * not break away at 1 kHz, raise it with 'maxd' -- short bench runs only. */
int  maxDuty = 145;
int  minDuty = 55;                      // below this a TT motor only buzzes
static const int RAMP_STEP = 6;         // duty counts per 10 ms

static const uint32_t DEADMAN_MS = 3000;
static const bool INVERT_LEFT  = false;
static const bool INVERT_RIGHT = false;

/* ========================================================================
 * ARM CONFIG -- bench-calibrated travel
 *   RACK 40..180 deg, INVERTED: 180 = RETRACTED (park), 40 = EXTENDED
 *   GRIP  0..60  deg:             0 = OPEN,             60 = CLOSED
 * ===================================================================== */
static const uint32_t SERVO_HZ  = 50;
static const uint8_t  SRV_BITS  = 16;
static const uint32_t SRV_FS    = (1UL << SRV_BITS) - 1;

// 500/2500 because the limits below were measured with that map. Changing
// one without re-measuring the other pushes the servo INTO its end stop.
int usMin = 500, usMax = 2500;

static const int RACK_EXTENDED  = 40;
static const int RACK_RETRACTED = 180;
static const int GRIP_OPEN      = 0;
static const int GRIP_CLOSED    = 60;

int slewDegPerSec = 90;
static const uint32_t SRV_TICK_MS = 20;

/* ========================================================================
 * STATE
 * ===================================================================== */
enum Side { LEFT = 0, RIGHT = 1 };
static const int EN_PIN[2] = {PIN_ENA, PIN_ENB};
static const int EN_CH[2]  = {CH_ENA,  CH_ENB};

int  tgtPct[2]  = {0, 0};
int  curDuty[2] = {0, 0};
int  curDir[2]  = {0, 0};
int  speedPct   = 60;
bool latchMode  = false;
uint32_t lastCmdMs = 0, lastRampMs = 0;

struct Arm {
  int  pin, ch;
  const char *name;
  float cur;
  int  tgt, lo, hi, home;
  bool attached;
};

Arm A[2] = {
  { PIN_RACK, CH_RACK, "rack", (float)RACK_RETRACTED, RACK_RETRACTED,
    RACK_EXTENDED, RACK_RETRACTED, RACK_RETRACTED, false },
  { PIN_GRIP, CH_GRIP, "grip", (float)GRIP_OPEN, GRIP_OPEN,
    GRIP_OPEN, GRIP_CLOSED, GRIP_OPEN, false },
};
static const int RACK = 0, GRIP = 1;
int  sel = RACK;
bool inSeq = false;                     // blocks nested sequences

/* Set by 'kill' and 'x'. Every blocking wait and every sequence loop honours
 * it, so the panic word reaches the arm from inside a 6-second servo wait
 * instead of being swallowed until the sequence's next step re-attaches. */
bool abortSeq = false;

/* True while the breakaway sweep owns the duty registers directly. drvRamp()
 * must not fight it -- it used to zero the duty and clear the dir pins every
 * 10 ms, so the sweep printed numbers that never reached the motors. */
bool drvManual = false;

/* Latched by bootPinReport() when a strapping pin reads high. A board that
 * cannot be trusted to reboot does not get to drive motors. */
bool driveLockout = false;

/* Start of the current non-zero drive command. Backstop against an indefinite
 * stall that outlives the dead-man, the latch and inSeq. */
uint32_t driveStartMs = 0;
static const uint32_t MAX_CONTINUOUS_DRIVE_MS = 20000;

/* Brake strength. Full duty on a Darlington bridge shorts the windings while
 * the motors still turn and dumps the regenerated current into the pack.
 * Tune on the bench with a clamp meter on the supply. */
static const int BRAKE_DUTY = 160;

/* Absolute mechanical envelope, from the bench stop-sweep. 'lim' may narrow
 * inside this; it may never widen past it. If a mechanism really travels
 * further, re-measure the stops and change the constants above deliberately. */
static const int HARD_LO[2] = {RACK_EXTENDED, GRIP_OPEN};
static const int HARD_HI[2] = {RACK_RETRACTED, GRIP_CLOSED};

/* ------------------------------------------------------------- forwards */
void pollSerial();
void serviceAll();
void waitMs(uint32_t ms);
void drvHardStop();
void armDetachAll();

/* ========================================================================
 * LEDC -- explicit channels on both cores
 * ===================================================================== */
// Returns the frequency the hardware actually achieved, 0 on failure. 'freq'
// checks this instead of reporting whatever the operator typed.
static uint32_t ledcSetupCh(int pin, int ch, uint32_t freq, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttachChannel(pin, freq, bits, ch) ? freq : 0;
#else
  uint32_t got = (uint32_t)ledcSetup(ch, freq, bits);
  ledcAttachPin(pin, ch);
  return got;
#endif
}
static void ledcDetachCh(int pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(pin);
#else
  ledcDetachPin(pin);
#endif
}
static void ledcWriteCh(int pin, int ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch; ledcWrite(pin, duty);
#else
  (void)pin; ledcWrite(ch, duty);
#endif
}

/* ========================================================================
 * DRIVE
 * ===================================================================== */
static void drvDirPins(Side s, int dir) {
  if ((s == LEFT) ? INVERT_LEFT : INVERT_RIGHT) dir = -dir;
  int a = (s == LEFT) ? PIN_IN1 : PIN_IN3;
  int b = (s == LEFT) ? PIN_IN2 : PIN_IN4;
  if (dir > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW);  }
  else if (dir < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else              { digitalWrite(a, LOW);  digitalWrite(b, LOW);  }
}

static void drvApply(Side s) {
  drvDirPins(s, curDir[s]);
  ledcWriteCh(EN_PIN[s], EN_CH[s], constrain(curDuty[s], 0, DRV_FS));
}

// Coast, THEN brake, and service the arm throughout.
//
// Jumping straight to both-inputs-high at full duty shorts the windings while
// the motors are still turning and dumps the regenerated current back into the
// pack. This is the path taken by 'x', 'kill', 'freq' and every dead-man
// timeout -- the most-executed code on the board -- so it must not be the
// thing that browns out the ESP32 mid-move.
//
// Deliberately does NOT call pollSerial(): drvHardStop() is itself reached
// from pollSerial() -> handleLine(), so polling here makes 'x' recurse without
// bound. serviceAll() only.
void drvHardStop() {
  // drvManual for the whole window: serviceAll() below runs drvRamp(), whose
  // drvApply() would clear the direction pins and zero EN on the next 10 ms
  // tick and quietly undo the brake.
  drvManual = true;

  for (int s = 0; s < 2; s++) {
    tgtPct[s] = curDuty[s] = curDir[s] = 0;
    ledcWriteCh(EN_PIN[s], EN_CH[s], 0);                 // coast first
  }
  uint32_t t0 = millis();
  while (millis() - t0 < 30) { serviceAll(); delay(2); }

  for (int s = 0; s < 2; s++) {
    int a = (s == LEFT) ? PIN_IN1 : PIN_IN3;
    int b = (s == LEFT) ? PIN_IN2 : PIN_IN4;
    digitalWrite(a, HIGH); digitalWrite(b, HIGH);        // both high = brake
    ledcWriteCh(EN_PIN[s], EN_CH[s], BRAKE_DUTY);
  }
  t0 = millis();
  while (millis() - t0 < 120) { serviceAll(); delay(2); }  // arm really does
                                                           // keep moving now
  for (int s = 0; s < 2; s++) {
    ledcWriteCh(EN_PIN[s], EN_CH[s], 0);
    drvDirPins((Side)s, 0);
  }
  drvManual = false;              // hand the registers back to drvRamp()
}

// Park every L298N control line LOW with LEDC DISCONNECTED. Setup-only, plus
// the 'drv' handler -- which must call drvReattachPwm() afterwards.
static void drvParkPins() {
  const int p[6] = {PIN_ENA, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENB};
  for (int i = 0; i < 6; i++) { pinMode(p[i], OUTPUT); digitalWrite(p[i], LOW); }
}

// Re-route ENA/ENB back to their LEDC channels.
//
// ANY pinMode(pin, INPUT*) writes SIG_GPIO_OUT_IDX into that pin's
// FUNC_OUT_SEL register -- see gpio_ll_output_disable() in the IDF GPIO HAL,
// comment "Ensure no other output signal is routed via GPIO matrix to this
// pin". That severs the LEDC route permanently, and pinMode(OUTPUT) does NOT
// restore it: the pin becomes a plain GPIO and ledcWrite() goes nowhere.
// bootPinReport() does exactly that to all eight pins, so before this existed
// typing 'pins' or 'drv' killed the drive until the next reset.
static void drvReattachPwm() {
  ledcSetupCh(PIN_ENA, CH_ENA, pwmFreqHz, DRV_BITS);
  ledcSetupCh(PIN_ENB, CH_ENB, pwmFreqHz, DRV_BITS);
  ledcWriteCh(PIN_ENA, CH_ENA, 0);
  ledcWriteCh(PIN_ENB, CH_ENB, 0);
}

static int drvDutyFromPct(int pct) {
  pct = constrain(abs(pct), 0, 100);
  return pct ? map(pct, 1, 100, minDuty, maxDuty) : 0;
}

static void drvRamp() {
  if (drvManual) return;          // the breakaway sweep owns the duty registers
  if (millis() - lastRampMs < 10) return;
  lastRampMs = millis();
  for (int s = 0; s < 2; s++) {
    int wantDir  = (tgtPct[s] > 0) ? 1 : (tgtPct[s] < 0 ? -1 : 0);
    int wantDuty = drvDutyFromPct(tgtPct[s]);
    // Never flip direction under load: ramp to zero first.
    if (wantDir != curDir[s] && curDuty[s] > 0) {
      curDuty[s] = max(0, curDuty[s] - RAMP_STEP * 2);
    } else {
      curDir[s] = wantDir;
      if (curDuty[s] < wantDuty)      curDuty[s] = min(wantDuty, curDuty[s] + RAMP_STEP);
      else if (curDuty[s] > wantDuty) curDuty[s] = max(wantDuty, curDuty[s] - RAMP_STEP);
    }
    drvApply((Side)s);
  }
}

static void drvCmd(int l, int r) {
  if (driveLockout) {
    Serial.println(F("  DRIVE LOCKED OUT: a strapping pin read high at boot.\n"
                     "  Fit the 10k pulldowns and reset before driving."));
    return;
  }
  bool wasMoving = (tgtPct[LEFT] || tgtPct[RIGHT]);
  tgtPct[LEFT]  = constrain(l, -100, 100);
  tgtPct[RIGHT] = constrain(r, -100, 100);
  if (!wasMoving && (tgtPct[LEFT] || tgtPct[RIGHT])) driveStartMs = millis();
  lastCmdMs = millis();           // ONLY a real motion command feeds the dead-man
}

/* ========================================================================
 * ARM
 * ===================================================================== */
static void armAttach(int i) {
  Arm &a = A[i];
  if (a.attached) return;
  ledcSetupCh(a.pin, a.ch, SERVO_HZ, SRV_BITS);
  a.attached = true;
}

static void armDetach(int i) {
  Arm &a = A[i];
  if (!a.attached) return;
  ledcDetachCh(a.pin);
  // Driven LOW, not high-Z. A continuous low is "no valid pulse" so the servo
  // goes limp just the same, but a floating signal line can pick up noise an
  // MG995 will act on.
  pinMode(a.pin, OUTPUT); digitalWrite(a.pin, LOW);
  a.attached = false;
}

void armDetachAll() {
  for (int i = 0; i < 2; i++) armDetach(i);
  Serial.println(F("  both servos DETACHED (limp, no holding torque)"));
}

static int armDegToUs(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return usMin + (int)((usMax - usMin) * deg / 180.0f);
}

static void armWriteUs(Arm &a, int us) {
  if (!a.attached) return;
  uint32_t duty = (uint32_t)((uint64_t)us * SRV_FS * SERVO_HZ / 1000000ULL);
  ledcWriteCh(a.pin, a.ch, duty);
}

static void armSetTarget(int i, int deg) {
  Arm &a = A[i];
  int c = constrain(deg, a.lo, a.hi);
  if (c != deg)
    Serial.printf("  !! %d deg outside %s limits [%d..%d] -> clamped to %d\n",
                  deg, a.name, a.lo, a.hi, c);
  a.tgt = c;
  // NO implicit armAttach(). Attaching writes a.cur -- the STALE assumed angle
  // -- straight to the LEDC register with no slew, because LEDC is hardware.
  // A servo you detached and repositioned by hand would snap back at full
  // speed. 'det -> set -> att' exists for this; the setter must not defeat it.
  if (!a.attached) {
    Serial.printf("  %s is DETACHED: target stored (%d deg), nothing will move.\n"
                  "  Its real angle is unknown -- 'set %c <deg>' then 'att'.\n",
                  a.name, c, a.name[0]);
    return;
  }
}

static void armSlew() {
  static uint32_t last = 0;
  if (millis() - last < SRV_TICK_MS) return;
  last = millis();
  float step = (float)slewDegPerSec * SRV_TICK_MS / 1000.0f;
  for (int i = 0; i < 2; i++) {
    Arm &a = A[i];
    if (!a.attached) continue;
    if (fabs(a.tgt - a.cur) < 0.05f)   a.cur = a.tgt;
    else if (a.cur < a.tgt)            a.cur = min((float)a.tgt, a.cur + step);
    else                               a.cur = max((float)a.tgt, a.cur - step);
    armWriteUs(a, armDegToUs(a.cur));
  }
}

static bool armMoving(int i) { return fabs(A[i].tgt - A[i].cur) > 0.05f; }

static void armNudge(int d) {
  Arm &a = A[sel];
  int want = a.tgt + d;
  if (want < a.lo || want > a.hi) {
    Serial.printf("  %s at a soft limit (%d). Widen with 'lim' only if the\n"
                  "  mechanism really travels further -- 2 deg at a time.\n",
                  a.name, a.tgt);
    return;
  }
  armSetTarget(sel, want);
  Serial.printf("  %s -> %d deg  (%d us)\n", a.name, a.tgt, armDegToUs((float)a.tgt));
}

// Declare the real shaft angle without moving. Firmware cannot read a servo.
static void armDeclare(int i, int deg) {
  Arm &a = A[i];
  if (a.attached) { Serial.println(F("  detach first ('det'), or it will jump")); return; }
  a.cur = a.tgt = constrain(deg, a.lo, a.hi);
  Serial.printf("  %s now assumed at %d deg (nothing moved)\n", a.name, a.tgt);
}

/* ========================================================================
 * COOPERATIVE CORE -- the reason both subsystems can run at once
 * ===================================================================== */
void serviceAll() { drvRamp(); armSlew(); }

// Wait that keeps BOTH ramp engines and the console alive, and that abandons
// the wait the moment 'kill' or 'x' is typed.
void waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    pollSerial(); serviceAll();
    if (abortSeq) return;
    delay(2);
  }
}

static void armWaitSettled(int i, uint32_t capMs) {
  uint32_t t0 = millis();
  // A[i].attached is in the condition because armSlew() skips detached servos,
  // so cur freezes and armMoving() stays true forever. Without it, every
  // 'kill' burned the full cap and then the next step re-attached and moved on.
  while (armMoving(i) && A[i].attached && millis() - t0 < capMs) {
    pollSerial(); serviceAll();
    if (abortSeq) return;
    delay(2);
  }
  if (A[i].attached && armMoving(i))
    Serial.printf("  !! %s did NOT reach %d deg in %lu ms (stuck at %.0f) -- binding?\n",
                  A[i].name, A[i].tgt, (unsigned long)capMs, A[i].cur);
}

/* ========================================================================
 * SEQUENCES  (drive stays commandable throughout)
 * ===================================================================== */
static void drvAutoSequence() {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  inSeq = true;
  Serial.println(F("\n=== DRIVE AUTO SEQUENCE (wheels off the ground) ==="));
  struct Step { int l, r; uint32_t ms; const char *label; };
  const Step st[] = {
    { speedPct,  0, 1500, "LEFT forward"},  {0, 0, 600, "pause"},
    {-speedPct,  0, 1500, "LEFT reverse"},  {0, 0, 600, "pause"},
    { 0,  speedPct, 1500, "RIGHT forward"}, {0, 0, 600, "pause"},
    { 0, -speedPct, 1500, "RIGHT reverse"}, {0, 0, 600, "pause"},
    { speedPct,  speedPct, 2000, "BOTH forward"},
    {-speedPct, -speedPct, 2000, "BOTH reverse"},
    { speedPct, -speedPct, 1500, "spin CW"},
    {-speedPct,  speedPct, 1500, "spin CCW"},
  };
  for (unsigned k = 0; k < sizeof(st)/sizeof(st[0]); k++) {
    if (abortSeq) break;
    Serial.printf("  -> %-16s L=%4d%%  R=%4d%%\n", st[k].label, st[k].l, st[k].r);
    drvCmd(st[k].l, st[k].r);
    uint32_t t0 = millis();
    while (millis() - t0 < st[k].ms) {
      lastCmdMs = millis(); pollSerial(); serviceAll();
      if (abortSeq) break;
      delay(2);
    }
  }
  drvHardStop();
  inSeq = false; abortSeq = false;
  Serial.println(F("=== SEQUENCE DONE ===\n"));
}

static void drvBreakaway() {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  if (driveLockout) { Serial.println(F("  drive is locked out, see the boot report")); return; }
  inSeq = true;
  // drvManual: without it, drvRamp() inside waitMs() zeroed the duty and
  // cleared the dir pins every 10 ms, so this swept nothing but the console.
  drvManual = true;
  Serial.printf("\n=== BREAKAWAY SWEEP, both sides forward, 20 -> %d ===\n", maxDuty);
  Serial.println(F("Note the duty where the wheels first turn steadily.\n"));
  drvDirPins(LEFT, 1); drvDirPins(RIGHT, 1);
  for (int d = 20; d <= maxDuty; d += 3) {
    if (abortSeq) break;
    ledcWriteCh(PIN_ENA, CH_ENA, d);
    ledcWriteCh(PIN_ENB, CH_ENB, d);
    Serial.printf("  duty = %3d / 255  (%2d%%)\n", d, (d * 100) / DRV_FS);
    waitMs(700);
  }
  drvManual = false;              // release before the stop path runs
  drvHardStop();
  inSeq = false; abortSeq = false;
  Serial.println(F("=== SWEEP DONE ===\n"));
}

static void armSweep(int i) {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  inSeq = true;
  Arm &a = A[i];
  Serial.printf("\n=== SWEEP %s : %d -> %d -> %d deg ===\n", a.name, a.lo, a.hi, a.home);
  Serial.println(F("  Listen for buzzing or grinding.\n"));
  armAttach(i);
  int seq[3] = {a.lo, a.hi, a.home};
  for (int k = 0; k < 3; k++) {
    if (abortSeq) break;
    Serial.printf("  -> %3d deg\n", seq[k]);
    armSetTarget(i, seq[k]);
    armWaitSettled(i, 6000);
  }
  inSeq = false; abortSeq = false;
  Serial.println(F("=== SWEEP DONE ===\n"));
}

static void armGripCycle() {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  inSeq = true;
  Serial.println(F("\n=== GRIPPER OPEN / CLOSE x3 ==="));
  armAttach(GRIP);                // explicit: armSetTarget no longer attaches
  for (int k = 0; k < 3; k++) {
    if (abortSeq) break;
    Serial.printf("  open  (%d)\n", GRIP_OPEN);
    armSetTarget(GRIP, GRIP_OPEN);   armWaitSettled(GRIP, 4000); waitMs(300);
    if (abortSeq) break;
    Serial.printf("  close (%d)\n", GRIP_CLOSED);
    armSetTarget(GRIP, GRIP_CLOSED); armWaitSettled(GRIP, 4000); waitMs(300);
  }
  inSeq = false; abortSeq = false;
  Serial.println(F("=== CYCLE DONE ===\n"));
}

static void armPickDemo() {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  inSeq = true;
  Serial.println(F("\n=== PICK-AND-PLACE MOTION, no cargo ==="));
  struct St { int ax, deg; const char *label; };
  const St st[] = {
    {GRIP, GRIP_OPEN,      "1. gripper open"},
    {RACK, RACK_EXTENDED,  "2. rack EXTEND  (40)"},
    {GRIP, GRIP_CLOSED,    "3. gripper close"},
    {RACK, RACK_RETRACTED, "4. rack RETRACT (180)"},
    {RACK, RACK_EXTENDED,  "5. rack EXTEND  (40)"},
    {GRIP, GRIP_OPEN,      "6. gripper open"},
    {RACK, RACK_RETRACTED, "7. rack RETRACT (180)"},
  };
  for (int i = 0; i < 2; i++) armAttach(i);   // explicit, see armSetTarget
  for (unsigned k = 0; k < sizeof(st)/sizeof(st[0]); k++) {
    if (abortSeq) break;
    Serial.printf("  %s\n", st[k].label);
    armSetTarget(st[k].ax, st[k].deg);
    armWaitSettled(st[k].ax, 6000);
    waitMs(300);
  }
  inSeq = false; abortSeq = false;
  Serial.println(F("=== DEMO DONE ===\n"));
}

// Release the load FIRST, then pull the rack in.
static void armPark() {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  inSeq = true;
  Serial.println(F("\n  parking: gripper open, then rack retracted"));
  for (int i = 0; i < 2; i++) armAttach(i);   // explicit, see armSetTarget
  armSetTarget(GRIP, GRIP_OPEN);      armWaitSettled(GRIP, 4000);
  if (!abortSeq) { armSetTarget(RACK, RACK_RETRACTED); armWaitSettled(RACK, 6000); }
  inSeq = false; abortSeq = false;
  Serial.println(F("  parked.\n"));
}

/* THE combined test: both subsystems moving at the same time. */
static void comboTest() {
  if (inSeq) { Serial.println(F("  a sequence is already running")); return; }
  inSeq = true;
  Serial.println(F("\n=== COMBINED LOAD TEST ==="));
  Serial.println(F("  Wheels driving WHILE the arm cycles. Watch for an ESP32"));
  Serial.println(F("  reset or a stutter -- that is the supply sagging, not code.\n"));

  Serial.println(F("  both wheels forward + gripper cycling"));
  for (int i = 0; i < 2; i++) armAttach(i);   // explicit, see armSetTarget
  drvCmd(speedPct, speedPct);
  for (int k = 0; k < 2 && !abortSeq; k++) {
    armSetTarget(GRIP, GRIP_CLOSED); armWaitSettled(GRIP, 4000);
    armSetTarget(GRIP, GRIP_OPEN);   armWaitSettled(GRIP, 4000);
    lastCmdMs = millis();
  }

  if (!abortSeq) {
    Serial.println(F("  spin CW + rack extend/retract (worst case for the pack)"));
    drvCmd(speedPct, -speedPct);
    armSetTarget(RACK, RACK_EXTENDED);  armWaitSettled(RACK, 6000);
    lastCmdMs = millis();
    armSetTarget(RACK, RACK_RETRACTED); armWaitSettled(RACK, 6000);
  }

  drvHardStop();
  inSeq = false; abortSeq = false;
  Serial.println(F("=== COMBINED TEST DONE ===\n"));
}

/* ========================================================================
 * REPORTS
 * ===================================================================== */
static void bootPinReport() {
  struct R { int gpio; const char *role; };
  const R rs[8] = {
    {PIN_ENA, "ENA left PWM   "}, {PIN_IN1, "IN1 left dir A "},
    {PIN_IN2, "IN2 left dir B "}, {PIN_IN3, "IN3 right dir A"},
    {PIN_IN4, "IN4 right dir B"}, {PIN_ENB, "ENB right PWM  "},
    {PIN_RACK,"RACK servo     "}, {PIN_GRIP,"GRIP servo     "},
  };
  // This report drives every pin as an input, which severs LEDC on all four
  // PWM pins (see drvReattachPwm) and leaves the servos limp while .attached
  // would still say "yes". Detach properly first, restore exactly what was
  // attached at the end.
  bool wasAttached[2] = { A[RACK].attached, A[GRIP].attached };
  for (int i = 0; i < 2; i++) armDetach(i);

  Serial.println(F("\n--------------- BOOT PIN REPORT ---------------"));
  Serial.println(F("  pin   role             up  dn  note"));
  for (int i = 0; i < 8; i++) {
    pinMode(rs[i].gpio, INPUT_PULLUP);   delayMicroseconds(800);
    int up = digitalRead(rs[i].gpio);
    pinMode(rs[i].gpio, INPUT_PULLDOWN); delayMicroseconds(800);
    int dn = digitalRead(rs[i].gpio);
    const char *note = "ok";
    if (dn == 1)      note = "** HELD HIGH - voltage feeding in **";
    else if (up == 0) note = "held low (normal with a 10k pulldown fitted)";
    if (rs[i].gpio == 12 || rs[i].gpio == 5 || rs[i].gpio == 2) {
      if (dn == 1) {
        note = "** STRAPPING PIN HELD HIGH - fix before driving **";
        driveLockout = true;          // refuse to drive, do not merely warn
      } else {
        note = "strapping pin, pulldown recommended";
      }
    }
    Serial.printf("  %-4d  %-15s  %d   %d   %s\n", rs[i].gpio, rs[i].role, up, dn, note);
    // Leave every pin driven LOW, servo lines included: a floating signal pin
    // is something an MG995 can act on.
    pinMode(rs[i].gpio, OUTPUT); digitalWrite(rs[i].gpio, LOW);
  }
  Serial.println(F("  ----------------------------------------------\n"));

  drvParkPins();
  drvReattachPwm();                   // without this the drive is dead
  for (int i = 0; i < 2; i++) if (wasAttached[i]) armAttach(i);
}

static void showState() {
  Serial.printf("\n  DRIVE  speed=%d%%  duty L=%d R=%d  dir L=%d R=%d  "
                "min=%d max=%d  latch=%s\n",
                speedPct, curDuty[LEFT], curDuty[RIGHT],
                curDir[LEFT], curDir[RIGHT], minDuty, maxDuty,
                latchMode ? "ON" : "off");
  Serial.printf("         PWM %d Hz  ->  at speed 100%% the duty is %d/255 (%d%%)\n",
                pwmFreqHz, maxDuty, (maxDuty * 100) / DRV_FS);
  Serial.println(F("  ARM    axis  pin  now   tgt   limits      home  attached"));
  for (int i = 0; i < 2; i++)
    Serial.printf("         %-4s  %-3d  %4.0f  %4d   [%3d..%3d]   %3d   %s\n",
                  A[i].name, A[i].pin, A[i].cur, A[i].tgt,
                  A[i].lo, A[i].hi, A[i].home, A[i].attached ? "yes" : "NO");
  Serial.printf("  pulse %d..%d us   slew %d deg/s   selected: %s\n\n",
                usMin, usMax, slewDegPerSec, A[sel].name);
}

static void printHelp() {
  Serial.println(F("\n=============== COMBINED DRIVE + ARM ==============="));
  Serial.println(F("  LINE-BUFFERED: type the command, press Enter."));
  Serial.println(F("  -- DRIVE ----------------------------------------"));
  Serial.println(F("   w  s        forward / reverse"));
  Serial.println(F("   a  d        spin CCW / spin CW"));
  Serial.println(F("   q  e        arc left / arc right"));
  Serial.println(F("   1  2        left side only / right side only"));
  Serial.println(F("   x           stop (brake)"));
  Serial.println(F("   +  -        speed +/- 10%"));
  Serial.println(F("   t           drive auto sequence"));
  Serial.println(F("   b           breakaway duty sweep"));
  Serial.println(F("   l           toggle dead-man latch"));
  Serial.println(F("   maxd <n>    duty clamp, 60..255   (pack voltage!)"));
  Serial.println(F("   freq <hz>   PWM frequency, 200..20000 (L298N likes 1000)"));
  Serial.println(F("   mind <n>    breakaway duty floor"));
  Serial.println(F("  -- ARM ------------------------------------------"));
  Serial.println(F("   r <deg>     rack  to angle   (40..180)"));
  Serial.println(F("   g <deg>     grip  to angle   (0..60)"));
  Serial.println(F("   ext  ret    rack extended (40) / retracted (180)"));
  Serial.println(F("   op   cl     gripper open (0) / closed (60)"));
  Serial.println(F("   sel r | g   choose servo for [ and ]"));
  Serial.println(F("   [  ]        nudge selected -2 / +2 deg"));
  Serial.println(F("   cen         both servos to home"));
  Serial.println(F("   det  att    detach (limp) / attach"));
  Serial.println(F("               arm commands do NOT auto-attach: 'att' first"));
  Serial.println(F("   hold        stop the arm where it is"));
  Serial.println(F("   sweep r|g   slow lo -> hi -> home"));
  Serial.println(F("   cycle       gripper open/close x3"));
  Serial.println(F("   demo        pick-and-place motion, no cargo"));
  Serial.println(F("   park        gripper open THEN rack retract"));
  Serial.println(F("   set r 180   declare real angle, no movement"));
  Serial.println(F("   lim r 40 180 / home r 180 / slew <d/s> / us 500 2500"));
  Serial.println(F("  -- BOTH -----------------------------------------"));
  Serial.println(F("   combo       drive AND arm together (load test)"));
  Serial.println(F("   kill        STOP wheels + DETACH servos + ABORT sequence"));
  Serial.println(F("   x           also aborts a running sequence"));
  Serial.println(F("   drv         re-park the L298N pins LOW"));
  Serial.println(F("   pins        boot pin report"));
  Serial.println(F("   st  ?       state / this help"));
  Serial.println(F("===================================================="));
  showState();
}

/* ========================================================================
 * PARSER
 * ===================================================================== */
static int axByLetter(char c) { return (c=='r') ? RACK : (c=='g') ? GRIP : -1; }

static void handleLine(char *line) {
  char v[12] = {0};
  int a = 0, b = 0;
  char w = 0;
  if (sscanf(line, "%11s", v) != 1) return;
  // NO blanket lastCmdMs refresh here. It used to be fed by every line typed,
  // including 'st', '?' and typos, so the wheels kept turning because you made
  // a typo and the dead-man protected nothing. Only drvCmd() refreshes it now.
  // To work the arm while the wheels turn, latch deliberately with 'l'.

  /* ---- drive, single letters ---- */
  if (!strcmp(v, "w")) { drvCmd( speedPct,  speedPct); Serial.println(F("FWD"));      return; }
  if (!strcmp(v, "s")) { drvCmd(-speedPct, -speedPct); Serial.println(F("REV"));      return; }
  if (!strcmp(v, "a")) { drvCmd(-speedPct,  speedPct); Serial.println(F("SPIN CCW")); return; }
  if (!strcmp(v, "d")) { drvCmd( speedPct, -speedPct); Serial.println(F("SPIN CW"));  return; }
  if (!strcmp(v, "q")) { drvCmd(speedPct/3, speedPct); Serial.println(F("ARC L"));    return; }
  if (!strcmp(v, "e")) { drvCmd(speedPct, speedPct/3); Serial.println(F("ARC R"));    return; }
  if (!strcmp(v, "1")) { drvCmd( speedPct, 0); Serial.println(F("LEFT only"));        return; }
  if (!strcmp(v, "2")) { drvCmd( 0, speedPct); Serial.println(F("RIGHT only"));       return; }
  if (!strcmp(v, "x")) { abortSeq = true; drvHardStop();
                         Serial.println(F("STOP"));                                   return; }
  if (!strcmp(v, "+")) { speedPct = min(100, speedPct + 10);
                         Serial.printf("speed %d%%\n", speedPct);                     return; }
  if (!strcmp(v, "-")) { speedPct = max(10,  speedPct - 10);
                         Serial.printf("speed %d%%\n", speedPct);                     return; }
  if (!strcmp(v, "t")) { drvAutoSequence(); return; }
  if (!strcmp(v, "b")) { drvBreakaway();    return; }
  if (!strcmp(v, "l")) { latchMode = !latchMode;
                         Serial.printf("latch %s\n", latchMode ? "ON" : "off");       return; }

  /* ---- both / safety ---- */
  if (!strcmp(v, "kill"))  { abortSeq = true; drvHardStop(); armDetachAll();
                             Serial.println(F("  KILLED: wheels stopped, arm limp, "
                                              "any sequence aborting"));              return; }
  if (!strcmp(v, "combo")) { comboTest();    return; }
  if (!strcmp(v, "drv"))   { drvHardStop(); drvParkPins(); drvReattachPwm();
                             Serial.println(F("  L298N pins forced LOW, PWM re-attached")); return; }
  if (!strcmp(v, "pins"))  { bootPinReport();   // restores pins and PWM itself
                             return; }
  if (!strcmp(v, "st"))    { showState();    return; }
  if (!strcmp(v, "?") || !strcmp(v, "help")) { printHelp(); return; }

  /* ---- arm ---- */
  if (!strcmp(v, "ext"))   { armSetTarget(RACK, RACK_EXTENDED);  return; }
  if (!strcmp(v, "ret"))   { armSetTarget(RACK, RACK_RETRACTED); return; }
  if (!strcmp(v, "op"))    { armSetTarget(GRIP, GRIP_OPEN);      return; }
  if (!strcmp(v, "cl"))    { armSetTarget(GRIP, GRIP_CLOSED);    return; }
  if (!strcmp(v, "cen"))   { for (int i=0;i<2;i++) armSetTarget(i, A[i].home); return; }
  if (!strcmp(v, "det"))   { armDetachAll(); return; }
  if (!strcmp(v, "att"))   { for (int i=0;i<2;i++) armAttach(i);
                             Serial.printf("  attached, holding rack=%.0f grip=%.0f\n"
                                           "  if that is not the real position: det, set, att\n",
                                           A[RACK].cur, A[GRIP].cur);                 return; }
  if (!strcmp(v, "hold"))  { for (int i=0;i<2;i++) A[i].tgt = (int)A[i].cur;
                             Serial.println(F("  arm holding position"));             return; }
  if (!strcmp(v, "cycle")) { armGripCycle(); return; }
  if (!strcmp(v, "demo"))  { armPickDemo();  return; }
  if (!strcmp(v, "park"))  { armPark();      return; }
  if (!strcmp(v, "["))     { armNudge(-2);   return; }
  if (!strcmp(v, "]"))     { armNudge(+2);   return; }

  if (!strcmp(v, "sel")) {
    if (sscanf(line, "%*s %c", &w) == 1 && axByLetter(w) >= 0) {
      sel = axByLetter(w); Serial.printf("  selected: %s\n", A[sel].name);
    } else Serial.println(F("  usage: sel r | sel g"));
    return;
  }
  if (!strcmp(v, "sweep")) {
    if (sscanf(line, "%*s %c", &w) == 1 && axByLetter(w) >= 0) armSweep(axByLetter(w));
    else Serial.println(F("  usage: sweep r | sweep g"));
    return;
  }
  if (!strcmp(v, "set")) {
    if (sscanf(line, "%*s %c %d", &w, &a) == 2 && axByLetter(w) >= 0) armDeclare(axByLetter(w), a);
    else Serial.println(F("  usage: set r 180 | set g 0"));
    return;
  }
  if (!strcmp(v, "lim")) {
    if (sscanf(line, "%*s %c %d %d", &w, &a, &b) == 3 && axByLetter(w) >= 0) {
      int i = axByLetter(w);
      if (a >= b) { Serial.println(F("  bad range")); return; }
      // Narrowing inside the measured envelope is fine. Widening past it
      // commands the servo into its end stop, where it stalls indefinitely --
      // there is no current sense and no stall timeout anywhere in here.
      if (a < HARD_LO[i] || b > HARD_HI[i]) {
        Serial.printf("  REFUSED. %s measured envelope is [%d..%d].\n"
                      "  Widening past it drives the servo into its end stop.\n"
                      "  Re-measure the stops, then change the constants in source.\n",
                      A[i].name, HARD_LO[i], HARD_HI[i]);
        return;
      }
      A[i].lo = a; A[i].hi = b; A[i].home = constrain(A[i].home, a, b);
      Serial.printf("  %s limits now [%d..%d]\n", A[i].name, a, b);
    } else Serial.println(F("  usage: lim r 40 180"));
    return;
  }
  if (!strcmp(v, "home")) {
    if (sscanf(line, "%*s %c %d", &w, &a) == 2 && axByLetter(w) >= 0) {
      int i = axByLetter(w);
      A[i].home = constrain(a, A[i].lo, A[i].hi);
      Serial.printf("  %s home now %d\n", A[i].name, A[i].home);
    } else Serial.println(F("  usage: home r 180"));
    return;
  }
  if (!strcmp(v, "slew")) {
    if (sscanf(line, "%*s %d", &a) == 1 && a >= 5 && a <= 400) {
      slewDegPerSec = a; Serial.printf("  slew = %d deg/s\n", a);
    } else Serial.println(F("  usage: slew 90   (5..400)"));
    return;
  }
  if (!strcmp(v, "us")) {
    if (sscanf(line, "%*s %d %d", &a, &b) == 2 && a >= 400 && b <= 2600 && a < b) {
      // This re-maps every angle, including the one each servo is holding
      // right now. Go limp on purpose rather than lurch into a stop.
      armDetachAll();
      usMin = a; usMax = b;
      Serial.printf("  pulse envelope %d..%d us. Both servos DETACHED deliberately:\n"
                    "  every angle now maps to a different pulse. Re-measure the\n"
                    "  stops, 'set' the real angles, then 'att'.\n", usMin, usMax);
    } else Serial.println(F("  usage: us 500 2500"));
    return;
  }
  if (!strcmp(v, "maxd")) {
    if (sscanf(line, "%*s %d", &a) == 1 && a >= 60 && a <= 255) {
      if (a <= minDuty) {
        Serial.printf("  REFUSED: maxd %d is at or below minDuty %d, which inverts\n"
                      "  the speed map (1%% would command more duty than 100%%).\n"
                      "  Lower 'mind' first.\n", a, minDuty);
        return;
      }
      maxDuty = a;
      // No voltage claimed here. This firmware does not know your pack, and
      // the old readout both hardcoded 12 V and ignored the ~2 V bridge drop.
      Serial.printf("  maxDuty = %d/255 (%d%%). Motor average = (Vpack - 2) * %d / 255.\n"
                    "  Size it from YOUR measured pack: maxd = 255 * 6 / (Vpack - 2).\n",
                    maxDuty, (maxDuty * 100) / DRV_FS, maxDuty);
    } else Serial.println(F("  usage: maxd 145   (60..255, above mind)"));
    return;
  }
  if (!strcmp(v, "freq")) {
    if (sscanf(line, "%*s %d", &a) == 1 && a >= 200 && a <= 20000) {
      drvHardStop();
      int want = a;
      pwmFreqHz = a;
      // Re-attach ONLY ch0/ch1 (timer0). The servo channels are on timer1
      // (timer = (chan/2)%4), so the arm keeps holding position across this.
      uint32_t got = ledcSetupCh(PIN_ENA, CH_ENA, pwmFreqHz, DRV_BITS);
      ledcSetupCh(PIN_ENB, CH_ENB, pwmFreqHz, DRV_BITS);
      ledcWriteCh(PIN_ENA, CH_ENA, 0);
      ledcWriteCh(PIN_ENB, CH_ENB, 0);
      if (got == 0) {
        Serial.printf("  !! hardware REJECTED %d Hz at %d-bit. Duty is 0; pick another.\n",
                      want, DRV_BITS);
      } else {
        pwmFreqHz = (int)got;     // report what the timer actually achieved
        Serial.printf("  PWM now %d Hz (asked %d). Lower is better on an L298N;\n"
                      "  500-1000 Hz gives the most low-speed torque.\n", pwmFreqHz, want);
      }
    } else Serial.println(F("  usage: freq 1000   (200..20000)"));
    return;
  }
  if (!strcmp(v, "mind")) {
    if (sscanf(line, "%*s %d", &a) == 1 && a >= 0 && a < maxDuty) {
      minDuty = a; Serial.printf("  minDuty = %d\n", minDuty);
    } else Serial.println(F("  usage: mind 55"));
    return;
  }

  /* r <deg> / g <deg>, or bare r / g to select */
  if (!strcmp(v, "r") || !strcmp(v, "g")) {
    int i = axByLetter(v[0]);
    if (sscanf(line, "%*s %d", &a) == 1) {
      armSetTarget(i, a);
      Serial.printf("  %s -> %d deg  (%d us)\n", A[i].name, A[i].tgt,
                    armDegToUs((float)A[i].tgt));
    } else { sel = i; Serial.printf("  selected: %s\n", A[i].name); }
    return;
  }

  Serial.printf("  unknown '%s'  (? for help)\n", v);
}

void pollSerial() {
  static char buf[48];
  static uint8_t n = 0;
  static bool ovf = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (ovf) {
        // Previously the tail past buf[47] silently became the NEXT command.
        Serial.println(F("  line too long -- DISCARDED, nothing executed"));
        ovf = false; n = 0;
      } else if (n) {
        buf[n] = 0; n = 0; handleLine(buf);
      }
    }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
    else ovf = true;
  }
}

/* ========================================================================
 * SETUP / LOOP
 * ===================================================================== */
void setup() {
  // FIRST: kill anything the boot pulse on GPIO5/GPIO14 left running.
  drvParkPins();

  // Servo signal lines driven LOW immediately. Left as floating inputs they
  // can feed an MG995 noise pulses between reset and the first 'att'.
  pinMode(PIN_RACK, OUTPUT); digitalWrite(PIN_RACK, LOW);
  pinMode(PIN_GRIP, OUTPUT); digitalWrite(PIN_GRIP, LOW);

  Serial.begin(115200);
  delay(400);
  Serial.println(F("\n\nESP32-B COMBINED drive + arm bring-up"));
  Serial.printf("Drive: ENA=%d IN1=%d IN2=%d IN3=%d IN4=%d ENB=%d  (ch%d/ch%d, %d Hz)\n",
                PIN_ENA, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENB,
                CH_ENA, CH_ENB, pwmFreqHz);
  Serial.printf("Arm  : RACK=%d GRIP=%d  (ch%d/ch%d, %lu Hz)\n",
                PIN_RACK, PIN_GRIP, CH_RACK, CH_GRIP, (unsigned long)SERVO_HZ);

  bootPinReport();                  // parks the pins and re-attaches PWM itself

  Serial.println(F("Servos start DETACHED, assumed at rack=180, grip=0."));
  Serial.println(F("  Arm commands no longer auto-attach. 'set' the real angles,"));
  Serial.println(F("  then 'att' -- otherwise attaching snaps the servo to a guess."));
  Serial.println(F("Separate 5-6 V servo supply, common ground. 'kill' is the panic word.\n"));

  printHelp();

  if (driveLockout)
    Serial.println(F("\n  *** DRIVE LOCKED OUT: a strapping pin read high at boot.\n"
                     "  *** GPIO12 high at reset sets VDD_SDIO to 1.8 V and a 3.3 V\n"
                     "  *** flash module may not survive the next reset. Fit the 10k\n"
                     "  *** pulldowns, then reset. No drive command will run.\n"));

  lastCmdMs = lastRampMs = driveStartMs = millis();
}

void loop() {
  pollSerial();
  serviceAll();

  // Dead-man: ONLY a real motion command refreshes this (see drvCmd). Silence
  // stops the wheels. 'l' latches past it, deliberately and visibly.
  if (!latchMode && !inSeq && (tgtPct[LEFT] || tgtPct[RIGHT]) &&
      (millis() - lastCmdMs > DEADMAN_MS)) {
    drvHardStop();
    Serial.println(F("[dead-man] no drive command -> wheels stopped"));
  }

  // Absolute backstop. Overrides latchMode AND inSeq, because neither is a
  // licence to hold a motor against a wall forever -- there is no current
  // sense on this board. The longest legitimate run is comboTest's ~12 s spin.
  if ((tgtPct[LEFT] || tgtPct[RIGHT]) &&
      (millis() - driveStartMs > MAX_CONTINUOUS_DRIVE_MS)) {
    drvHardStop();
    Serial.printf("[stall guard] %lu s of continuous drive -> stopped\n",
                  (unsigned long)(MAX_CONTINUOUS_DRIVE_MS / 1000));
  }
}
