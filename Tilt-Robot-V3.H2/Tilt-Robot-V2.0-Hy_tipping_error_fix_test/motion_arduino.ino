// motion_arduino.ino — Arduino motion controller for the tilt-table station.
//
// Implements the MOTION half of docs/PROTOCOL.md: parses "#<tag> VERB args"
// lines from the laptop over USB serial, drives the two Zaber stages through the
// X-MCC (via the X-AS01 shield over I2C), and reports status/events. Motion is
// non-blocking: MOVE/HOME return "OK ACCEPTED" immediately, then the sketch
// polls the stages and emits "EVT DONE <axis>" on completion.
//
// The hardware E-stop is wired directly into the X-MCC and cuts motor power
// without software. This sketch only OBSERVES the resulting state (via a sense
// line and/or Zaber alerts) and emits "EVT ESTOP" so the laptop can abort. It
// never commands the E-stop.
//
// ==== INTEGRATION TODOs (the hardware-specific parts) ====
//  1. Uses Zaber's official ASCII library (Shield/Connection over the X-AS01 I2C
//     bridge). Install it via Arduino Library Manager ("Zaber ASCII"). Confirm
//     the shield's I2C address jumpers match ZABERSHIELD_ADDRESS_AA (0x90).
//  2. Fill STEPS_PER_DEG for each stage from its datasheet (microsteps/rev / 360).
//  3. Confirm the X-MCC device/axis addressing (OUTER_DEV/AX, INNER_DEV/AX) and
//     the ESTOP sense pin wiring.

#include <ZaberShield.h>
#include <ZaberConnection.h>
#include <ZaberCommand.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace Zaber;

// ------------------------------------------------------------------ config --
static const unsigned long USB_BAUD   = 115200;
static const int     ESTOP_SENSE_PIN  = 2;      // digital sense of X-MCC E-stop

// Zaber addressing — CURRENT HARDWARE (2026-07-18, bench-confirmed): TWO
// RDQ150-AE57T10A stages on the one X-MCC. Axis 2 carries the mechanically-
// OUTER stage; axis 1 carries the (motion-limited) INNER stage.
static const uint8_t OUTER_DEV = 1, OUTER_AX = 2;   // RDQ150 (outer)
static const uint8_t INNER_DEV = 1, INNER_AX = 1;   // RDQ150 (inner, limited)

// The X-AS01 shield speaks I2C to the Arduino — set its jumpers to this address.
Shield shield(ZABERSHIELD_ADDRESS_AA);   // 0x90
Connection connection(shield);

// Probed from the device itself via the slider GUI hint line (identical
// stages -> identical conversion). Re-confirm via the hint line if in doubt.
static const double OUTER_STEPS_PER_DEG = 360000.0;   // RDQ150 (probed)
static const double INNER_STEPS_PER_DEG = 360000.0;   // RDQ150 (same model)

// Axis presence — probed at boot (a stage may be absent, e.g. RMA'd). Absent
// axes are skipped by HOME/MOVEB, report nan positions, and reject MOVE.
static bool outerPresent = false;
static bool innerPresent = false;

// ---- Direction-aware homing (collision-sensitive fixture) ------------------
// The fixture has a NO-GO arc beyond ~+/-135 deg. Plain `home` searches
// negative/CCW from anywhere and can sweep through it — so this firmware NEVER
// issues plain home. The laptop chooses the search direction from a trusted
// angle and sends `HOME <axis> <NEG|POS>`; we run
// `tools gotolimit home <dir> 2 0` under a relative-travel + time watchdog
// (unhomed position deltas are trustworthy; absolutes are not), then verify
// limit.home.triggered + warning flags before declaring the axis homed.
static const double        HOME_TRAVEL_CAP_DEG = 150.0;  // > 130 + offset margin
static const unsigned long HOME_TIMEOUT_MS     = 90000UL;
static bool   homedOuter = false, homedInner = false;
static bool   homingIsOuter = false;        // which axis the active HOME drives
static double homingStartDeg = 0.0;         // raw pos at search start
static unsigned long homingDeadline = 0;

// Travel limits, degrees (full +/-180 per the design).
static const double OUTER_MIN = -180.0, OUTER_MAX = 180.0;
static const double INNER_MIN = -180.0, INNER_MAX = 180.0;

static const unsigned long POLL_INTERVAL_MS = 25;  // motion-done poll cadence

// ------------------------------------------------------------------- state --
enum State { S_IDLE, S_MOVING, S_HOMING, S_FAULT, S_ESTOP };
static State state = S_IDLE;
static bool  homed = false;
static char  doneAxis[8] = "ALL";
static bool  estopAnnounced = false;
static unsigned long lastPoll = 0;

// USB line assembly
static char lineBuf[96];
static uint8_t lineLen = 0;

// ------------------------------------------------------- response emitters --
static void sendOk(long tag, const char* payload) {
  Serial.print('#'); Serial.print(tag); Serial.print(" OK");
  if (payload && payload[0]) { Serial.print(' '); Serial.print(payload); }
  Serial.print('\n');
}
static void sendErr(long tag, const char* code, const char* msg) {
  Serial.print('#'); Serial.print(tag); Serial.print(" ERR ");
  Serial.print(code); Serial.print(' '); Serial.print(msg); Serial.print('\n');
}
static void sendEvt(const char* name, const char* payload) {
  Serial.print("EVT "); Serial.print(name);
  if (payload && payload[0]) { Serial.print(' '); Serial.print(payload); }
  Serial.print('\n');
}

// ============================================================================
//  Zaber motion — via Zaber's official ASCII library over the X-AS01 (I2C)
// ============================================================================
// True if the given device/axis reports IDLE (motion complete). NON-BLOCKING:
// getStatus() sends one status query and reads its reply, so the loop stays
// responsive to USB commands during a move (unlike the blocking waitUntilIdle).
static bool zaberIdle(uint8_t dev, uint8_t axis) {
  return connection.getStatus(dev, axis) == Result::IDLE;
}

// Per-axis travel windows, read from the devices at boot. A factory rotary
// addresses 0..360, but once limit.min/limit.max are configured (the inner
// axis's no-go restriction) the window may be signed or wrapped — a blanket
// deg%360 then commands out-of-range targets the device rejects.
static double axLimLoDeg[2] = {0.0, 0.0};       // [0]=outer, [1]=inner
static double axLimHiDeg[2] = {360.0, 360.0};

// Map a +/-180 protocol target into the axis's window; NAN if unreachable.
static double mapTargetToLimits(double signedDeg, double lo, double hi) {
  const double eps = 1e-6;
  double cands[3] = {signedDeg, signedDeg + 360.0, signedDeg - 360.0};
  for (int i = 0; i < 3; i++) {
    if (cands[i] >= lo - eps && cands[i] <= hi + eps) return cands[i];
  }
  return NAN;
}

static double wrapTo180(double deg) {
  double d = fmod(deg + 180.0, 360.0);
  if (d < 0) d += 360.0;
  return d - 180.0;
}

// Raw (unwrapped) position in degrees — valid for RELATIVE deltas even when
// unhomed, which is what the homing watchdog needs.
static double zaberGetPosRawDeg(uint8_t dev, uint8_t axis, double stepsPerDeg) {
  Result r = connection.genericCommand("get pos", dev, axis);
  return (double)r.getDataInt() / stepsPerDeg;
}

// Current position in degrees via "get pos" (reply data is microsteps).
static double zaberGetPosDeg(uint8_t dev, uint8_t axis, double stepsPerDeg) {
  return wrapTo180(zaberGetPosRawDeg(dev, axis, stepsPerDeg));
}

// axIdx: 0 = outer, 1 = inner (selects the travel-window row).
// Returns false when the target has no representation inside the window.
static bool zaberMoveDeg(uint8_t dev, uint8_t axis, int axIdx, double deg,
                         double stepsPerDeg) {
  double devDeg = mapTargetToLimits(deg, axLimLoDeg[axIdx], axLimHiDeg[axIdx]);
  if (isnan(devDeg)) return false;
  long steps = (long)(devDeg * stepsPerDeg);
  // CHECK the device's reply. A rejected move used to be silently ignored,
  // which turned config mismatches into phantom "OK + DONE" no-ops.
  Result r = connection.genericCommand(Command("move abs ", steps), dev, axis);
  if (r.getError() != Result::OK || r.getIsRejected()) return false;
  return true;
}

// Read an axis's configured travel window (degrees) into the tables.
static void readAxisLimits(int axIdx, uint8_t dev, uint8_t axis,
                           double stepsPerDeg) {
  Result lo = connection.genericCommand("get limit.min", dev, axis);
  Result hi = connection.genericCommand("get limit.max", dev, axis);
  if (lo.getError() == Result::OK && hi.getError() == Result::OK &&
      !lo.getIsRejected() && !hi.getIsRejected()) {
    axLimLoDeg[axIdx] = (double)lo.getDataInt() / stepsPerDeg;
    axLimHiDeg[axIdx] = (double)hi.getDataInt() / stepsPerDeg;
  } else {
    axLimLoDeg[axIdx] = 0.0;      // fall back to the factory 0..360 window
    axLimHiDeg[axIdx] = 360.0;
  }
}

// NOTE: there is deliberately NO zaberHome() issuing plain `home` — see the
// direction-aware homing block above. Blind home can sweep the no-go arc.
static void recomputeHomed() {
  bool any = outerPresent || innerPresent;
  homed = any
      && (!outerPresent || homedOuter)
      && (!innerPresent || homedInner);
}

static void zaberStop(uint8_t dev, uint8_t axis) {
  connection.genericCommand("stop", dev, axis);
}

// True if a working stage answers on this device/axis. An empty X-MCC slot
// ("Unused") rejects "get pos", which is how we detect a missing stage.
static bool probeAxis(uint8_t dev, uint8_t axis) {
  Result r = connection.genericCommand("get pos", dev, axis);
  return r.getError() == Result::OK && !r.getIsRejected();
}

// Portable float formatting via integer math — classic AVR snprintf can't do
// %f (prints '?') and dtostrf availability varies by core. This works
// identically on Uno R3, Mega, and Uno R4 (Renesas).
static char* fmtDeg(double v, char* buf, size_t n) {
  if (isnan(v)) {
    strncpy(buf, "nan", n);
    buf[n - 1] = '\0';
    return buf;
  }
  long milli = lround(v * 1000.0);        // +/-180000 fits comfortably in long
  snprintf(buf, n, "%s%ld.%03ld", (milli < 0 ? "-" : ""),
           labs(milli) / 1000L, labs(milli) % 1000L);
  return buf;
}

// ---------------------------------------------------------------- helpers ---
static bool inRange(double v, double lo, double hi) { return v >= lo && v <= hi; }

static void beginMove(const char* axis) {
  strncpy(doneAxis, axis, sizeof(doneAxis) - 1);
  doneAxis[sizeof(doneAxis) - 1] = '\0';
  state = S_MOVING;
}

// Poll the stages; when both idle, finish the move and emit EVT DONE.
// HOMING is single-axis and guarded: a relative-travel + time watchdog stops
// the search before it can reach the no-go arc, and success requires
// limit.home.triggered with clean warning flags.
static void checkMotionDone() {
  if (millis() - lastPoll < POLL_INTERVAL_MS) return;
  lastPoll = millis();

  if (state == S_HOMING) {
    uint8_t dev  = homingIsOuter ? OUTER_DEV : INNER_DEV;
    uint8_t axn  = homingIsOuter ? OUTER_AX  : INNER_AX;
    double  spd  = homingIsOuter ? OUTER_STEPS_PER_DEG : INNER_STEPS_PER_DEG;

    // wrapTo180 on the DIFFERENCE: raw pos runs 0..360, so a naive subtraction
    // across the 360->0 boundary (or the position reset that homing performs)
    // reports a phantom ~360 deg jump and trips the cap on a *successful* home.
    // Wrapping gives the true shortest angular distance travelled.
    double nowRaw   = zaberGetPosRawDeg(dev, axn, spd);
    double traveled = fabs(wrapTo180(nowRaw - homingStartDeg));
    if (traveled > HOME_TRAVEL_CAP_DEG || millis() > homingDeadline) {
      zaberStop(dev, axn);
      state = S_IDLE;
      // Instrumented ("v2"): report the numbers that tripped the cap, so a
      // fault tells us WHICH condition fired and with what values. If a fault
      // ever arrives without "v2", the board is running an old build.
      char tb[12], sb[12], nb[12], msg[96];
      snprintf(msg, sizeof(msg), "home watchdog v2 trav=%s start=%s now=%s tleft=%ld",
               fmtDeg(traveled, tb, sizeof(tb)),
               fmtDeg(homingStartDeg, sb, sizeof(sb)),
               fmtDeg(nowRaw, nb, sizeof(nb)),
               (long)(homingDeadline - millis()));
      sendEvt("FAULT", msg);
      return;
    }
    if (zaberIdle(dev, axn)) {
      // Verify: index found AND no "no reference" warning remaining.
      Result trig = connection.genericCommand("get limit.home.triggered",
                                              dev, axn);
      Result st = connection.genericCommand("", dev, axn);
      bool ok = trig.getDataInt() == 1 && !st.getHasWarning();
      if (ok) {
        if (homingIsOuter) homedOuter = true; else homedInner = true;
        recomputeHomed();
        state = S_IDLE;
        sendEvt("DONE", doneAxis);
      } else {
        state = S_IDLE;
        sendEvt("FAULT", "home verify failed (limit.home.triggered/warnings)");
      }
    }
    return;
  }

  bool outerIdle = !outerPresent || zaberIdle(OUTER_DEV, OUTER_AX);
  bool innerIdle = !innerPresent || zaberIdle(INNER_DEV, INNER_AX);
  if (outerIdle && innerIdle) {
    state = S_IDLE;
    sendEvt("DONE", doneAxis);
  }
}

static void checkEstop() {
  // Active-low sense: LOW means E-stop asserted (TODO: match your wiring).
  if (digitalRead(ESTOP_SENSE_PIN) == LOW) {
    if (state != S_ESTOP) { state = S_ESTOP; }
    if (!estopAnnounced) { sendEvt("ESTOP", ""); estopAnnounced = true; }
  } else {
    estopAnnounced = false;  // re-arm once cleared
    if (state == S_ESTOP) state = S_IDLE;
  }
}

static const char* stateName(State s) {
  switch (s) {
    case S_IDLE:   return "IDLE";
    case S_MOVING: return "MOVING";
    case S_HOMING: return "HOMING";
    case S_FAULT:  return "FAULT";
    case S_ESTOP:  return "ESTOP";
  }
  return "IDLE";
}

// -------------------------------------------------------- command dispatch --
static void handleCommand(long tag, char* rest) {
  char* verb = strtok(rest, " ");
  if (!verb) { sendErr(tag, "EBADCMD", "empty"); return; }

  if (strcmp(verb, "PING") == 0) { sendOk(tag, "PONG"); return; }

  if (strcmp(verb, "STATUS") == 0) {
    double o = outerPresent
        ? zaberGetPosDeg(OUTER_DEV, OUTER_AX, OUTER_STEPS_PER_DEG) : NAN;
    double i = innerPresent
        ? zaberGetPosDeg(INNER_DEV, INNER_AX, INNER_STEPS_PER_DEG) : NAN;
    char ob[16], ib[16], buf[128];
    // fw=3 identifies this build over the wire (wrap-fix + v2 watchdog + move
    // reply-check). If STATUS lacks fw=3, the board is running an older build.
    snprintf(buf, sizeof(buf),
             "%s outer=%s inner=%s homed=%d outer_ok=%d inner_ok=%d "
             "outer_homed=%d inner_homed=%d fw=3",
             stateName(state), fmtDeg(o, ob, sizeof(ob)),
             fmtDeg(i, ib, sizeof(ib)), homed ? 1 : 0,
             outerPresent ? 1 : 0, innerPresent ? 1 : 0,
             homedOuter ? 1 : 0, homedInner ? 1 : 0);
    sendOk(tag, buf);
    return;
  }

  if (strcmp(verb, "GETPOS") == 0) {
    double o = outerPresent
        ? zaberGetPosDeg(OUTER_DEV, OUTER_AX, OUTER_STEPS_PER_DEG) : NAN;
    double i = innerPresent
        ? zaberGetPosDeg(INNER_DEV, INNER_AX, INNER_STEPS_PER_DEG) : NAN;
    char ob[16], ib[16], buf[48];
    snprintf(buf, sizeof(buf), "outer=%s inner=%s",
             fmtDeg(o, ob, sizeof(ob)), fmtDeg(i, ib, sizeof(ib)));
    sendOk(tag, buf);
    return;
  }

  if (strcmp(verb, "STOP") == 0) {
    zaberStop(OUTER_DEV, OUTER_AX); zaberStop(INNER_DEV, INNER_AX);
    state = S_IDLE;
    sendOk(tag, "STOPPED");
    return;
  }

  if (state == S_ESTOP) { sendErr(tag, "EESTOP", "station in E-stop"); return; }

  if (strcmp(verb, "HOME") == 0) {
    // Direction-aware ONLY: HOME <OUTER|INNER> <NEG|POS>. One axis at a time,
    // direction chosen by the laptop from a trusted angle. Plain/blind homing
    // is rejected — it could sweep the fixture's no-go arc.
    const char* axis = strtok(NULL, " ");
    const char* dirS = strtok(NULL, " ");
    if (!axis || !dirS) {
      sendErr(tag, "EARG",
              "HOME <OUTER|INNER> <NEG|POS> (direction-aware fixture)");
      return;
    }
    bool isOuter = strcmp(axis, "OUTER") == 0;
    bool isInner = strcmp(axis, "INNER") == 0;
    if (!isOuter && !isInner) { sendErr(tag, "EARG", "axis OUTER|INNER"); return; }
    if (isOuter && !outerPresent) { sendErr(tag, "ENOAXIS", "outer not connected"); return; }
    if (isInner && !innerPresent) { sendErr(tag, "ENOAXIS", "inner not connected"); return; }
    const char* dir = NULL;
    if (strcmp(dirS, "NEG") == 0) dir = "neg";
    else if (strcmp(dirS, "POS") == 0) dir = "pos";
    else { sendErr(tag, "EARG", "direction NEG|POS"); return; }
    if (state == S_MOVING || state == S_HOMING) { sendErr(tag, "EBUSY", "moving"); return; }

    uint8_t dev = isOuter ? OUTER_DEV : INNER_DEV;
    uint8_t axn = isOuter ? OUTER_AX  : INNER_AX;
    double  spd = isOuter ? OUTER_STEPS_PER_DEG : INNER_STEPS_PER_DEG;

    homingIsOuter = isOuter;
    if (isOuter) homedOuter = false; else homedInner = false;
    recomputeHomed();
    homingStartDeg = zaberGetPosRawDeg(dev, axn, spd);
    homingDeadline = millis() + HOME_TIMEOUT_MS;

    char cmd[40];
    snprintf(cmd, sizeof(cmd), "tools gotolimit home %s 2 0", dir);
    connection.genericCommand(cmd, dev, axn);

    beginMove(axis);
    state = S_HOMING;
    sendOk(tag, "ACCEPTED");
    return;
  }

  if (strcmp(verb, "MOVE") == 0) {
    if (!homed)          { sendErr(tag, "ENOTHOMED", "home first"); return; }
    if (state == S_MOVING || state == S_HOMING) { sendErr(tag, "EBUSY", "moving"); return; }
    char* axis = strtok(NULL, " ");
    char* degS = strtok(NULL, " ");
    if (!axis || !degS)  { sendErr(tag, "EARG", "MOVE axis deg"); return; }
    double deg = atof(degS);
    if (strcmp(axis, "OUTER") == 0) {
      if (!outerPresent) { sendErr(tag, "ENOAXIS", "outer not connected"); return; }
      if (!inRange(deg, OUTER_MIN, OUTER_MAX)) { sendErr(tag, "ELIMIT", "outer"); return; }
      if (!zaberMoveDeg(OUTER_DEV, OUTER_AX, 0, deg, OUTER_STEPS_PER_DEG)) {
        sendErr(tag, "ELIMIT", "outer: outside axis travel window"); return;
      }
    } else if (strcmp(axis, "INNER") == 0) {
      if (!innerPresent) { sendErr(tag, "ENOAXIS", "inner not connected"); return; }
      if (!inRange(deg, INNER_MIN, INNER_MAX)) { sendErr(tag, "ELIMIT", "inner"); return; }
      if (!zaberMoveDeg(INNER_DEV, INNER_AX, 1, deg, INNER_STEPS_PER_DEG)) {
        sendErr(tag, "ELIMIT", "inner: outside axis travel window"); return;
      }
    } else { sendErr(tag, "EARG", "axis OUTER|INNER"); return; }
    beginMove(axis);
    sendOk(tag, "ACCEPTED");
    return;
  }

  if (strcmp(verb, "MOVEB") == 0) {
    if (!homed)          { sendErr(tag, "ENOTHOMED", "home first"); return; }
    if (state == S_MOVING || state == S_HOMING) { sendErr(tag, "EBUSY", "moving"); return; }
    char* oS = strtok(NULL, " ");
    char* iS = strtok(NULL, " ");
    if (!oS || !iS)      { sendErr(tag, "EARG", "MOVEB outer inner"); return; }
    double o = atof(oS), i = atof(iS);
    if (!inRange(o, OUTER_MIN, OUTER_MAX) || !inRange(i, INNER_MIN, INNER_MAX)) {
      sendErr(tag, "ELIMIT", "range"); return;
    }
    // Atomic: verify BOTH targets are reachable before moving anything.
    if (outerPresent &&
        isnan(mapTargetToLimits(o, axLimLoDeg[0], axLimHiDeg[0]))) {
      sendErr(tag, "ELIMIT", "outer: outside axis travel window"); return;
    }
    if (innerPresent &&
        isnan(mapTargetToLimits(i, axLimLoDeg[1], axLimHiDeg[1]))) {
      sendErr(tag, "ELIMIT", "inner: outside axis travel window"); return;
    }
    // Best-effort with a missing stage: move whichever axes exist.
    if (outerPresent) zaberMoveDeg(OUTER_DEV, OUTER_AX, 0, o, OUTER_STEPS_PER_DEG);
    if (innerPresent) zaberMoveDeg(INNER_DEV, INNER_AX, 1, i, INNER_STEPS_PER_DEG);
    beginMove("ALL");
    sendOk(tag, "ACCEPTED");
    return;
  }

  sendErr(tag, "EBADCMD", verb);
}

// Parse a full "#<tag> VERB args" line.
static void processLine(char* line) {
  if (line[0] != '#') { sendErr(0, "EBADCMD", "no tag"); return; }
  char* sp = strchr(line, ' ');
  if (!sp) { sendErr(0, "EBADCMD", "no verb"); return; }
  *sp = '\0';
  long tag = atol(line + 1);
  handleCommand(tag, sp + 1);
}

// --------------------------------------------------------------- USB read ---
static void readUsb() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) processLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

// ----------------------------------------------------------------- Arduino --
void setup() {
  Serial.begin(USB_BAUD);
  shield.begin();               // I2C to the X-AS01 UART bridge -> X-MCC @115200
  connection.setTimeout(250);   // don't let a missing reply hang the poll loop
  pinMode(ESTOP_SENSE_PIN, INPUT_PULLUP);

  // Detect which stages are actually connected (works with 1 or 2), then
  // read each one's configured travel window for target mapping.
  delay(200);
  outerPresent = probeAxis(OUTER_DEV, OUTER_AX);
  innerPresent = probeAxis(INNER_DEV, INNER_AX);
  if (outerPresent) readAxisLimits(0, OUTER_DEV, OUTER_AX, OUTER_STEPS_PER_DEG);
  if (innerPresent) readAxisLimits(1, INNER_DEV, INNER_AX, INNER_STEPS_PER_DEG);
}

void loop() {
  checkEstop();
  if (state == S_MOVING || state == S_HOMING) checkMotionDone();
  readUsb();
}
