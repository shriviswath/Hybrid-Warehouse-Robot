/* ============================================================================
 *  HYBRID WAREHOUSE ROBOT  --  unified phone control  (ESP32-B master)
 * ----------------------------------------------------------------------------
 *  GROUND mode : wheels (L298N) + rack/gripper servos, driven from the phone
 *  FLIGHT mode : phone sticks -> MAVLink RC_CHANNELS_OVERRIDE -> ArduPilot
 *                on a Pixhawk 2.4.8 (TELEM2), which does all stabilisation.
 *
 *  Phone connects to the SoftAP below and opens http://192.168.4.1
 *  Works on Android, iPhone and a laptop. "Add to Home Screen" = app.
 *
 * ============================================================================
 *  DEPENDENCIES
 *    1. "WebSockets" by Markus Sattler          (Library Manager)
 *    2. MAVLink C library v2:
 *         https://github.com/mavlink/c_library_v2  -> download as ZIP
 *         unzip into  Arduino/libraries/c_library_v2/
 *       This sketch includes <common/mavlink.h> from it.
 *
 * ============================================================================
 *  ARDUPILOT SETUP THAT MUST BE DONE FIRST, IN QGROUNDCONTROL / MISSION PLANNER
 *  This firmware will not work, and is not safe, until these are set:
 *
 *    FRAME_CLASS   = 1      (Quad)
 *    FRAME_TYPE    = 1      (X)   -- or 0 for Plus, match your build
 *    accel + compass calibration done
 *    motor test from the GCS matches your physical motor order
 *
 *    RC1..RC4 trims/ranges 1000-2000 (override uses raw PWM)
 *    RC5_OPTION or FLTMODE channel as you prefer -- this sketch sets modes
 *                                                   with SET_MODE, not a switch
 *    FS_THR_ENABLE = 3      LAND on link loss (1 = RTL only with a GPS)
 *    RC_OVERRIDE_TIME = 1.0 (seconds) so a stalled link drops override fast
 *    BATT_MONITOR + BATT_LOW_VOLT configured, or you get no battery telemetry
 *
 *  PIXHAWK TELEM2 WIRING AND PARAMETERS
 *    TELEM2 pin 2 (TX) -> ESP32-B GPIO34      pin 3 (RX) <- ESP32-B GPIO13
 *    TELEM2 pin 6 (GND) -> ESP32-B GND        DO NOT connect pin 1 (5V)
 *
 *    SERIAL2_PROTOCOL = 2     MAVLink 2
 *    SERIAL2_BAUD     = 115   115200
 *    BRD_SER2_RTSCTS  = 0     no flow control wires fitted -- without this
 *                             the port can stay completely silent
 *
 *    RC1/2/4: MIN 1000 MAX 2000 TRIM 1500   RC3: MIN 1000 MAX 2000 TRIM 1000
 *    (set by hand: there is no transmitter to run radio calibration with)
 *
 *  >>> FIRST FLIGHTS ON A TETHER, PROPS BALANCED, IN AN OPEN SPACE. <<<
 *  >>> Fly in ALT_HOLD. STABILIZE is for people with a real transmitter. <<<
 * ========================================================================= */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <common/mavlink.h>
#include "webui_fly.h"

/* ------------------------------------------------------------------- WIFI */
static const char *AP_SSID = "HybridBot";
static const char *AP_PASS = "warehouse2026";
static const uint8_t AP_CHANNEL = 6;

/* ------------------------------------------------------------------- PINS */
static const int PIN_ENA = 5;
static const int PIN_IN1 = 12;
static const int PIN_IN2 = 14;
static const int PIN_IN3 = 27;
static const int PIN_IN4 = 26;
static const int PIN_ENB = 25;

static const int PIN_RACK = 2;
static const int PIN_GRIP = 4;

static const int PIN_UART_RX = 34;      // from ESP32-A TX
static const int PIN_UART_TX = 13;      // to   ESP32-A RX
static const uint32_t MAV_BAUD = 115200;

/* ------------------------------------------------------- LEDC CHANNELS --
 * ch0/ch1 -> timer0 : 1 kHz, 8-bit  (drive)
 * ch2/ch3 -> timer1 : 50 Hz, 16-bit (servos)
 * Explicit, never auto-allocated: a servo landing on timer0 would turn the
 * motor PWM into a 50 Hz chopper.
 * ---------------------------------------------------------------------- */
static const int CH_ENA = 0, CH_ENB = 1, CH_RACK = 2, CH_GRIP = 3;
static const int DRV_BITS = 8, DRV_FS = 255;
static const uint32_t SRV_HZ = 50;
static const uint8_t  SRV_BITS = 16;
static const uint32_t SRV_FS = (1UL << SRV_BITS) - 1;

/* --------------------------------------------------------- ARM GEOMETRY */
static const int RACK_EXTENDED = 40, RACK_RETRACTED = 180;

/* Travel range of the gripper servo, and which end of it is which.
 * Measured on the bench: 60 deg is OPEN, 0 deg is CLOSED. Keep MIN/MAX in
 * numeric order for the clamp; OPEN/CLOSED carry the meaning. */
static const int GRIP_MIN = 0,  GRIP_MAX = 60;
static const int GRIP_OPEN = 60, GRIP_CLOSED = 0;

/* Arm slew rate, degrees per second.
 *   GROUND  fast: nothing is balancing, snap moves are fine.
 *   FLIGHT  slow: arm travel moves the CoG, and the attitude controller has
 *           to absorb it. A 140 deg rack stroke at 25 deg/s takes ~5.6 s,
 *           which is a disturbance the controller can track. The same stroke
 *           at 90 deg/s is an impulse it has to catch. Raise this only after
 *           a successful hover with the arm moving. */
static const float ARM_RATE_GROUND = 90.0f;
static const float ARM_RATE_FLIGHT = 25.0f;
static const int SRV_US_MIN = 500,   SRV_US_MAX = 2500;

/* ------------------------------------------------------- DRIVE CFG ------
 * All tunable live from the phone, so the weak-motor hunt needs no reflash.
 *   drvMaxDuty  ceiling at 100% stick. Size it to the pack:
 *                 255 * 6 / (Vpack - 2)   -> 145 on 11.1 V, 255 on 7.4 V
 *   dutyFloor   below this a TT gearmotor only buzzes
 *   deadband    stick slop around centre, out of 100
 *   pwmFreqHz   L298N is a Darlington bridge: 500-1000 Hz has the most
 *               low-speed torque, above ~5 kHz it falls away
 * ---------------------------------------------------------------------- */
int  drvMaxDuty  = 255;
int  dutyFloor   = 60;
int  deadband    = 8;
int  pwmFreqHz   = 1000;
/* Both sides measured reversed on this build: stick-up drove the robot
 * backwards. Inverting both corrects travel direction and brings the
 * steering sense with it. Flip either one live from the phone if a motor
 * is ever rewired. */
bool invertL     = true;
bool invertR     = true;
static const int SLEW_PER_TICK = 12;

/* ------------------------------------------------------------ FAILSAFE --
 * Phone link is NOT flight-critical by design: ArduPilot owns the aircraft.
 * But a dead link must not leave the last stick values commanding the motors.
 * ---------------------------------------------------------------------- */
static const uint16_t LINK_LOST_MS   = 500;    // stop sending overrides
static const uint16_t LINK_LAND_MS   = 2000;   // then command LAND
static const uint16_t GROUND_STOP_MS = 400;    // wheels stop

/* ------------------------------------------------- COPTER CUSTOM MODES -- */
enum CopterMode : uint32_t {
  CM_STABILIZE = 0, CM_ALT_HOLD = 2, CM_LOITER = 5,
  CM_RTL = 6, CM_LAND = 9, CM_POSHOLD = 16
};

/* ------------------------------------------------------------------ MAV */
static const uint8_t GCS_SYSID  = 255;
static const uint8_t GCS_COMPID = MAV_COMP_ID_MISSIONPLANNER;
static uint8_t  apSysId  = 1;                  // learned from its heartbeat
static uint8_t  apCompId = MAV_COMP_ID_AUTOPILOT1;
static bool     apSeen   = false;

/* ---------------------------------------------------------------- STATE */
WebServer        http(80);
WebSocketsServer ws(81);

enum Mode : uint8_t { MODE_GROUND = 0, MODE_FLIGHT = 1 };
volatile Mode mode = MODE_GROUND;
volatile bool estop = true;                    // latched at boot

// ground
volatile int8_t cmdThrottle = 0, cmdSteer = 0;
volatile uint8_t cmdRack = RACK_RETRACTED, cmdGrip = GRIP_OPEN;   // starts open
int16_t dutyL = 0, dutyR = 0;
float   curRack = RACK_RETRACTED, curGrip = GRIP_OPEN;
bool    servosLive = false;
bool    armLocked  = true;      // controls read-only outside GROUND mode

// flight sticks, -100..100 each (throttle too: 0 = centre = hold)
volatile int8_t stkRoll = 0, stkPitch = 0, stkYaw = 0, stkThr = -100;
volatile bool   overrideOn = false;

// telemetry from the autopilot
float    apRoll = 0, apPitch = 0, apYaw = 0, apAlt = 0, apClimb = 0;
float    apVolt = 0, apCurrent = 0;
int      apBattPct = -1;
uint32_t apCustomMode = CM_STABILIZE;
bool     apArmed = false;
uint32_t lastHeartbeatMs = 0;

uint32_t lastPacketMs = 0;                     // last phone packet
bool     landCommanded = false;

/* MAVLink link diagnostics -- printed every 2 s on the USB serial.
 * rxBytes climbing but rxMsgs stuck  -> wrong baud, or noise on the wire
 * rxBytes stuck at 0                 -> nothing arriving: TX/RX swapped,
 *                                       wrong port, no common ground, or
 *                                       SERIAL2_PROTOCOL not set to 2 */
uint32_t rxBytes = 0, rxMsgs = 0, rxHeartbeats = 0;

/* ========================================================================
 * LEDC helpers
 * ===================================================================== */
static void pwmSetup(int pin, int ch, uint32_t freq, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, freq, bits, ch);
#else
  ledcSetup(ch, freq, bits); ledcAttachPin(pin, ch);
#endif
}
static void pwmWrite(int pin, int ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch; ledcWrite(pin, duty);
#else
  (void)pin; ledcWrite(ch, duty);
#endif
}
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ========================================================================
 * GROUND DRIVE
 * ===================================================================== */
static void driveChannel(int pinEn, int ch, int pinA, int pinB, int16_t d, bool inv) {
  if (d == 0) { digitalWrite(pinA, LOW); digitalWrite(pinB, LOW); pwmWrite(pinEn, ch, 0); return; }
  bool fwd = inv ? (d < 0) : (d > 0);
  digitalWrite(pinA, fwd ? HIGH : LOW);
  digitalWrite(pinB, fwd ? LOW  : HIGH);
  pwmWrite(pinEn, ch, (uint32_t)abs(d));
}

static void allStop() {
  dutyL = dutyR = 0;
  driveChannel(PIN_ENA, CH_ENA, PIN_IN1, PIN_IN2, 0, invertL);
  driveChannel(PIN_ENB, CH_ENB, PIN_IN3, PIN_IN4, 0, invertR);
}

// Change the drive PWM frequency without touching timer1 (the servos).
static void pwmSetFreq(int pin, int ch, uint32_t freq, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcChangeFrequency(pin, freq, bits);
#else
  (void)pin; ledcSetup(ch, freq, bits);
#endif
}

static void parkDrivePins() {
  const int p[6] = {PIN_ENA, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENB};
  for (int i = 0; i < 6; i++) { pinMode(p[i], OUTPUT); digitalWrite(p[i], LOW); }
}

static void updateDrive() {
  if (estop || mode != MODE_GROUND) { allStop(); return; }
  int thr = cmdThrottle, str = cmdSteer;
  if (abs(thr) < deadband) thr = 0;
  if (abs(str) < deadband) str = 0;

  int l = thr + str, r = thr - str;
  int m = max(abs(l), abs(r));
  if (m > 100) { l = l * 100 / m; r = r * 100 / m; }

  const int span = drvMaxDuty - dutyFloor;
  int tl = l == 0 ? 0 : (l > 0 ?  (dutyFloor + l * span / 100) : -(dutyFloor - l * span / 100));
  int tr = r == 0 ? 0 : (r > 0 ?  (dutyFloor + r * span / 100) : -(dutyFloor - r * span / 100));

  dutyL += clampi(tl - dutyL, -SLEW_PER_TICK, SLEW_PER_TICK);
  dutyR += clampi(tr - dutyR, -SLEW_PER_TICK, SLEW_PER_TICK);
  driveChannel(PIN_ENA, CH_ENA, PIN_IN1, PIN_IN2, dutyL, invertL);
  driveChannel(PIN_ENB, CH_ENB, PIN_IN3, PIN_IN4, dutyR, invertR);
}

/* ========================================================================
 * ARM SERVOS -- locked whenever we are not in GROUND mode
 * ===================================================================== */
static uint32_t servoDuty(float deg) {
  float us = SRV_US_MIN + (SRV_US_MAX - SRV_US_MIN) * deg / 180.0f;
  return (uint32_t)(us * SRV_FS * SRV_HZ / 1000000.0f);
}

static void servosAttach() {
  if (servosLive) return;
  pwmSetup(PIN_RACK, CH_RACK, SRV_HZ, SRV_BITS);
  pwmSetup(PIN_GRIP, CH_GRIP, SRV_HZ, SRV_BITS);
  servosLive = true;
}

/* ARM POLICY -- aerial manipulation is an objective, so the arm works in
 * BOTH modes. Three rules make that survivable:
 *
 *   RATE     GROUND moves at 90 deg/s, FLIGHT at 25 deg/s. Arm travel shifts
 *            the CoG; a slow move is a disturbance the attitude controller
 *            tracks, a fast one is an impulse it has to catch
 *            (Ruggiero et al., aerial manipulation review).
 *
 *   HOLDING  the servos are never detached, in any mode. A rack-and-pinion
 *            gripper back-drives, so a limp servo drops the cargo.
 *
 *   E-STOP   the only thing that freezes the controls. Even then the servos
 *            keep holding their last angle.
 */
static void updateServos() {
  servosAttach();                                 // never detached in flight

  bool allow = !estop;
  if (allow == armLocked) {                       // state changed
    armLocked = !allow;
    Serial.printf("[arm] %s\n", allow ? "controls live" : "frozen, holding position");
  }

  if (allow) {
    float rate = (mode == MODE_FLIGHT) ? ARM_RATE_FLIGHT : ARM_RATE_GROUND;
    const float step = rate * 20.0f / 1000.0f;    // per 20 ms tick
    float tr = clampi(cmdRack, RACK_EXTENDED, RACK_RETRACTED);
    float tg = clampi(cmdGrip, GRIP_MIN,      GRIP_MAX);
    curRack += clampi((int)((tr - curRack) * 10), -(int)(step * 10), (int)(step * 10)) / 10.0f;
    curGrip += clampi((int)((tg - curGrip) * 10), -(int)(step * 10), (int)(step * 10)) / 10.0f;
  }

  // Written every pass in every mode -- this is what keeps the cargo gripped.
  pwmWrite(PIN_RACK, CH_RACK, servoDuty(curRack));
  pwmWrite(PIN_GRIP, CH_GRIP, servoDuty(curGrip));
}

/* ========================================================================
 * MAVLINK
 * ===================================================================== */
static void mavSend(mavlink_message_t &msg) {
  static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
}

static void mavHeartbeat() {
  mavlink_message_t m;
  mavlink_msg_heartbeat_pack(GCS_SYSID, GCS_COMPID, &m,
                             MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  mavSend(m);
}

// Stick values -> raw PWM. 1500 is centre for roll/pitch/yaw, and in ALT_HOLD
// or LOITER 1500 throttle means HOLD ALTITUDE -- which is what makes a
// touchscreen throttle usable at all.
static uint16_t stickToPwm(int8_t v) { return (uint16_t)clampi(1500 + v * 5, 1000, 2000); }

static void mavSendOverride() {
  mavlink_message_t m;
  mavlink_msg_rc_channels_override_pack(
      GCS_SYSID, GCS_COMPID, &m, apSysId, apCompId,
      stickToPwm(stkRoll),            // ch1 roll
      stickToPwm(stkPitch),           // ch2 pitch
      stickToPwm(stkThr),             // ch3 throttle
      stickToPwm(stkYaw),             // ch4 yaw
      0, 0, 0, 0,                     // ch5-8  : 0 = no override
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0);  // ch9-18 : 0 = no override
  mavSend(m);
}

// Releasing every channel hands control back to ArduPilot's own failsafe.
static void mavReleaseOverride() {
  mavlink_message_t m;
  mavlink_msg_rc_channels_override_pack(GCS_SYSID, GCS_COMPID, &m, apSysId, apCompId,
      0,0,0,0, 0,0,0,0, 0,0,0,0,0,0,0,0,0,0);
  mavSend(m);
}

static void mavSetMode(uint32_t customMode) {
  mavlink_message_t m;
  mavlink_msg_set_mode_pack(GCS_SYSID, GCS_COMPID, &m, apSysId,
                            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, customMode);
  mavSend(m);
}

/* ArduPilot sends HEARTBEAT on a telemetry port unprompted, but ATTITUDE,
 * VFR_HUD and SYS_STATUS only if a ground station asks. Without this the
 * link shows as UP while battery, altitude and climb all read zero.
 * Re-sent periodically: the Pixhawk may boot after us, or miss the first. */
static void mavRequestStreams() {
  struct Req { uint8_t stream; uint16_t hz; };
  static const Req reqs[] = {
    { MAV_DATA_STREAM_EXTENDED_STATUS, 2  },   // SYS_STATUS  -> battery
    { MAV_DATA_STREAM_EXTRA1,          10 },   // ATTITUDE    -> roll, pitch
    { MAV_DATA_STREAM_EXTRA2,          4  },   // VFR_HUD     -> alt, climb
  };
  for (unsigned i = 0; i < sizeof(reqs) / sizeof(reqs[0]); i++) {
    mavlink_message_t m;
    mavlink_msg_request_data_stream_pack(GCS_SYSID, GCS_COMPID, &m,
                                         apSysId, apCompId,
                                         reqs[i].stream, reqs[i].hz, 1);
    mavSend(m);
  }
}

static void mavArm(bool arm) {
  mavlink_message_t m;
  mavlink_msg_command_long_pack(GCS_SYSID, GCS_COMPID, &m, apSysId, apCompId,
                                MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                arm ? 1.0f : 0.0f, 0, 0, 0, 0, 0, 0);
  mavSend(m);
}

static void mavPoll() {
  mavlink_message_t msg;
  mavlink_status_t  st;
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    rxBytes++;
    if (!mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &st)) continue;
    rxMsgs++;

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        if (msg.compid != MAV_COMP_ID_AUTOPILOT1) break;
        mavlink_heartbeat_t hb; mavlink_msg_heartbeat_decode(&msg, &hb);
        apSysId  = msg.sysid;
        apCompId = msg.compid;
        apSeen   = true;
        apArmed  = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        apCustomMode = hb.custom_mode;
        rxHeartbeats++;
        lastHeartbeatMs = millis();
        break;
      }
      case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a; mavlink_msg_attitude_decode(&msg, &a);
        apRoll  = a.roll  * 57.2958f;
        apPitch = a.pitch * 57.2958f;
        apYaw   = a.yaw   * 57.2958f;
        break;
      }
      case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t v; mavlink_msg_vfr_hud_decode(&msg, &v);
        apAlt   = v.alt;
        apClimb = v.climb;
        break;
      }
      case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t s; mavlink_msg_sys_status_decode(&msg, &s);
        apVolt    = s.voltage_battery / 1000.0f;
        apCurrent = s.current_battery / 100.0f;
        apBattPct = s.battery_remaining;
        break;
      }
      default: break;
    }
  }
}

/* ========================================================================
 * PHONE PROTOCOL
 *   D,<thr>,<steer>            ground drive, -100..100
 *   S,<rack>,<grip>            servo angles
 *   F,<roll>,<pitch>,<yaw>,<thr>   flight sticks, -100..100
 *   M,<0|1>                    mode: ground / flight
 *   E,<0|1>                    e-stop
 *   A,<1|0>                    arm / disarm the autopilot
 *   C,<alt|loi|rtl|land|stab>  flight mode
 *   H                          heartbeat
 * ===================================================================== */
static void enterMode(Mode want) {
  if (want == mode) return;
  allStop();
  cmdThrottle = cmdSteer = 0;
  if (want == MODE_FLIGHT) {
    // the arm stays live: it only locks once the autopilot is ARMED
    stkRoll = stkPitch = stkYaw = 0;
    stkThr = -100;                  // throttle at minimum on entry, always
    overrideOn = false;
  } else {
    overrideOn = false;
    mavReleaseOverride();
  }
  mode = want;
}

static void handleText(const char *msg) {
  lastPacketMs = millis();
  landCommanded = false;
  switch (msg[0]) {
    case 'D': { int t, s;
      if (sscanf(msg + 2, "%d,%d", &t, &s) == 2) {
        cmdThrottle = clampi(t, -100, 100); cmdSteer = clampi(s, -100, 100); } break; }
    case 'S': { int a, b;                        // arm works in BOTH modes
      if (sscanf(msg + 2, "%d,%d", &a, &b) == 2) {
        cmdRack = clampi(a, RACK_EXTENDED, RACK_RETRACTED);
        cmdGrip = clampi(b, GRIP_MIN, GRIP_MAX); } break; }
    case 'F': { int r, p, y, t;
      if (mode == MODE_FLIGHT && sscanf(msg + 2, "%d,%d,%d,%d", &r, &p, &y, &t) == 4) {
        stkRoll = clampi(r, -100, 100); stkPitch = clampi(p, -100, 100);
        stkYaw  = clampi(y, -100, 100); stkThr   = clampi(t, -100, 100);
        overrideOn = true; } break; }
    case 'M': enterMode(msg[2] == '1' ? MODE_FLIGHT : MODE_GROUND); break;
    case 'E':
      estop = (msg[2] == '1');
      if (estop) { allStop(); if (mode == MODE_FLIGHT) { mavSetMode(CM_LAND); overrideOn = false; } }
      break;
    case 'A':
      if (mode == MODE_FLIGHT && !estop) mavArm(msg[2] == '1');
      break;
    case 'C': {
      if (mode != MODE_FLIGHT || estop) break;
      const char *m = msg + 2;
      if      (!strncmp(m, "alt",  3)) mavSetMode(CM_ALT_HOLD);
      else if (!strncmp(m, "loi",  3)) mavSetMode(CM_LOITER);
      else if (!strncmp(m, "rtl",  3)) mavSetMode(CM_RTL);
      else if (!strncmp(m, "land", 4)) mavSetMode(CM_LAND);
      else if (!strncmp(m, "stab", 4)) mavSetMode(CM_STABILIZE);
      break;
    }
    /* G,<maxDuty>,<floor>,<deadband>   live drive tuning */
    case 'G': { int a, b, c;
      if (sscanf(msg + 2, "%d,%d,%d", &a, &b, &c) == 3) {
        drvMaxDuty = clampi(a, 60, 255);
        dutyFloor  = clampi(b, 0, drvMaxDuty - 10);
        deadband   = clampi(c, 0, 40);
        Serial.printf("[drv] max %d floor %d dead %d\n", drvMaxDuty, dutyFloor, deadband);
      } break; }
    /* I,<invL>,<invR>   flip a side that runs backwards */
    case 'I': { int a, b;
      if (sscanf(msg + 2, "%d,%d", &a, &b) == 2) {
        allStop();
        invertL = a; invertR = b;
        Serial.printf("[drv] invert L%d R%d\n", invertL, invertR);
      } break; }
    /* Q,<hz>   PWM frequency */
    case 'Q': { int a;
      if (sscanf(msg + 2, "%d", &a) == 1 && a >= 200 && a <= 20000) {
        allStop();
        pwmFreqHz = a;
        pwmSetFreq(PIN_ENA, CH_ENA, pwmFreqHz, DRV_BITS);
        pwmSetFreq(PIN_ENB, CH_ENB, pwmFreqHz, DRV_BITS);
        Serial.printf("[drv] pwm %d Hz\n", pwmFreqHz);
      } break; }
    case 'H': default: break;
  }
}

static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      lastPacketMs = millis();
      Serial.printf("[ws] client %u up\n", num);
      break;
    case WStype_DISCONNECTED:
      allStop(); cmdThrottle = cmdSteer = 0;
      Serial.printf("[ws] client %u gone\n", num);
      break;
    case WStype_TEXT: {
      char buf[72];
      size_t n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, payload, n); buf[n] = 0;
      handleText(buf);
      break;
    }
    default: break;
  }
}

static const char *modeName(uint32_t m) {
  switch (m) {
    case CM_STABILIZE: return "STABILIZE";
    case CM_ALT_HOLD:  return "ALT_HOLD";
    case CM_LOITER:    return "LOITER";
    case CM_RTL:       return "RTL";
    case CM_LAND:      return "LAND";
    case CM_POSHOLD:   return "POSHOLD";
    default:           return "MODE?";
  }
}

static void sendTelemetry() {
  bool apLive = apSeen && (millis() - lastHeartbeatMs < 3000);
  char buf[224];
  snprintf(buf, sizeof(buf),
           "T,%u,%u,%u,%u,%s,%.1f,%d,%.1f,%.1f,%.0f,%.0f,%d,%d,%d,%u,%u,%u,"
           "%d,%d,%d,%u,%u,%d",
           (unsigned)mode, (unsigned)estop, (unsigned)apArmed, (unsigned)apLive,
           modeName(apCustomMode), apVolt, apBattPct, apAlt, apClimb,
           apRoll, apPitch, dutyL, dutyR, (int)WiFi.softAPgetStationNum(),
           (unsigned)armLocked, (unsigned)curRack, (unsigned)curGrip,
           drvMaxDuty, dutyFloor, deadband,
           (unsigned)invertL, (unsigned)invertR, pwmFreqHz);
  ws.broadcastTXT(buf);
}

/* ========================================================================
 * SETUP / LOOP
 * ===================================================================== */
void setup() {
  parkDrivePins();                       // kill any boot-pulse motor output

  Serial.begin(115200);
  Serial2.begin(MAV_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  delay(200);
  Serial.println("\n[boot] ESP32-B master, MAVLink to ESP32-A");

  pwmSetup(PIN_ENA, CH_ENA, pwmFreqHz, DRV_BITS);
  pwmSetup(PIN_ENB, CH_ENB, pwmFreqHz, DRV_BITS);
  allStop();
  servosAttach();
  pwmWrite(PIN_RACK, CH_RACK, servoDuty(curRack));
  pwmWrite(PIN_GRIP, CH_GRIP, servoDuty(curGrip));

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 2);
  WiFi.setSleep(false);                  // latency: never let the radio nap
  Serial.print("[wifi] http://"); Serial.println(WiFi.softAPIP());

  http.on("/", HTTP_GET, []() {
    http.sendHeader("Cache-Control", "no-store");
    http.send_P(200, "text/html", INDEX_HTML);
  });
  http.on("/manifest.json", HTTP_GET, []() {
    http.send_P(200, "application/manifest+json", MANIFEST_JSON);
  });
  http.begin();

  ws.begin();
  ws.onEvent(onWsEvent);
  lastPacketMs = millis();
}

void loop() {
  http.handleClient();
  ws.loop();
  mavPoll();

  uint32_t now = millis();
  static uint32_t tCtl = 0, tTel = 0, tHb = 0, tOvr = 0;

  if (now - tHb >= 1000) { tHb = now; mavHeartbeat(); }

  // Ask the Pixhawk for telemetry streams, and keep asking every 5 s.
  static uint32_t tStream = 0;
  if (apSeen && now - tStream >= 5000) { tStream = now; mavRequestStreams(); }

  /* ---- phone link failsafe ---- */
  uint32_t quiet = now - lastPacketMs;
  if (quiet > GROUND_STOP_MS) { cmdThrottle = cmdSteer = 0; }

  if (mode == MODE_FLIGHT) {
    if (quiet > LINK_LOST_MS && overrideOn) {
      // Stop pretending to be a receiver. ArduPilot's own throttle failsafe
      // takes the aircraft -- that is what it is for.
      overrideOn = false;
      mavReleaseOverride();
      Serial.println("[fs] phone link lost, override released");
    }
    if (quiet > LINK_LAND_MS && apArmed && !landCommanded) {
      mavSetMode(CM_LAND);              // belt and braces
      landCommanded = true;
      Serial.println("[fs] link still down, commanding LAND");
    }
  }

  if (now - tCtl >= 20) {               // 50 Hz
    tCtl = now;
    updateDrive();
    updateServos();
  }

  if (mode == MODE_FLIGHT && overrideOn && !estop && now - tOvr >= 50) {
    tOvr = now;                         // 20 Hz override stream
    mavSendOverride();
  }

  if (now - tTel >= 200) { tTel = now; sendTelemetry(); }

  static uint32_t tDiag = 0;
  if (now - tDiag >= 2000) {
    tDiag = now;
    Serial.printf("[mav] rx %lu bytes  %lu msgs  %lu hb  %s   "
                  "mode %s  armed %d  batt %.1fV  alt %.1fm\n",
                  (unsigned long)rxBytes, (unsigned long)rxMsgs,
                  (unsigned long)rxHeartbeats,
                  (apSeen && now - lastHeartbeatMs < 3000) ? "LINK UP" : "NO LINK",
                  modeName(apCustomMode), (int)apArmed, apVolt, apAlt);
  }
}
