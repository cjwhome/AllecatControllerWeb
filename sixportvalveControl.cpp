#include <Arduino.h>

#include <EEPROM.h>

enum Phase
{
  PHASE_FILL,
  PHASE_INJECTION
};

// Track current phase so we can move open-loop without constant checks
static bool currentPhaseKnown = false;
static Phase currentPhase = PHASE_FILL; // arbitrary default

// === New calibration & runtime tracking ===
static const uint8_t CAL_TRIALS = 3;
static const uint8_t CAL_TOL_STEPS = 15; // max difference allowed between trials

volatile uint16_t stepsFillToInject = 0; // center→center FILL -> INJECT (forward)
volatile uint16_t stepsInjectToFill = 0; // center→center INJECT -> FILL (reverse)
volatile bool calCentersValid = false;

// Dwell at the FILL phase during Continuous mode (ms)
uint32_t dwellMs = 1; // default 500 ms

// Dwell at the INJECTION phase during Continuous mode (ms).
// Kept separate from the FILL dwell (dwellMs) so injection timing can be tuned
// independently. Live-settable while continuous mode is running via the serial
// command "j<ms>\n" (see pollAbortKey); also via the 'J' menu command.
uint32_t injectDwellMs = 1;

// Step pulse period in microseconds (interval between pulses) for MICROSTEP mode.
// Lower = faster motion. Tunable live via the 'P' command.
// 700us at 1/8 microstep ~= 1430 microsteps/sec ~= 178 full-sps equivalent shaft speed.
uint16_t stepPeriodUs = 700;

// Step period for FULL-STEP mode (calibration, F/f/I/i seeks). Without an
// acceleration ramp the motor stalls if you push full-step too fast — keep
// this conservative. ~2500us = 400 sps, comfortably within stall margin.
const uint16_t FULL_STEP_PERIOD_US = 2500;

// Set to 8 while running 1/8 microstep so step-count limits and the
// calibrated bump (measured in full-steps) scale correctly.
uint8_t microstepMultiplier = 1;

// How often to report sensor states during stepping
const uint16_t REPORT_EVERY = 60;

// ==== Motor control pins on RedBoard (as you already had) ====
#define stp 2
#define dir 3
#define MS1 4
#define MS2 5
#define EN 6

// ==== Optical sensor inputs (use analog pins as digital w/ pullups) ====

#define SENSOR1 7  // white wire
#define SENSOR2 8  // black wirex
#define SENSOR3 9  // orange wire
#define SENSOR4 10 // yellow wire

// Declare variables for functions
char user_input;
int x;
int y;
int state;

// === Add near your other globals ===
const uint16_t CHECK_EVERY = 20;         // check sensors every 20 steps while hunting
const uint32_t MAX_STEPS_TO_FIND = 8000; // safety cap so we don't spin forever
volatile bool abortRequested = false;

// Calibration storage (volatile for ISR-safety if you later add interrupts)
volatile uint16_t fillWidthSteps = 0;
volatile uint16_t injectWidthSteps = 0;
volatile uint16_t fillBumpSteps = 0;   // = fillWidthSteps/2
volatile uint16_t injectBumpSteps = 0; // = injectWidthSteps/2

// Limits for calibration scans
const uint32_t CALIB_SEARCH_LIMIT = 12000; // max steps to find a phase edge
const uint32_t CALIB_MAX_WIDTH = 12000;    // guard against runaway if stuck in phase

enum FindResult
{
  FOUND,
  NOT_FOUND,
  ABORTED
};

// Prototypes
void resetEDPins();
void StepForwardDefault();
void ReverseStepDefault();
void SmallStepModeForward();
void SmallStepModeReverse();
void ForwardBackwardStep();
static inline void stepOnce();
static inline void stepOnceAndReport(uint16_t idx);
static inline bool sensorsMatchPhase(Phase p);
static inline void printSensorStates(uint16_t stepIndex);
void goToPhase(Phase target, bool forward);
void GoToFillForward();
void GoToFillReverse();
void GoToInjectForward();
void GoToInjectReverse();
static inline void printSnapshot(uint32_t stepCount, const char *labelDir, const char *labelPhase);
FindResult goToPhaseAbortable(Phase target, bool forward);
void ContinuousFillInject();
static inline bool pollAbortKey();
static inline void stepN(uint32_t n);
FindResult stepUntilPhaseEnter(Phase target, bool forward, uint32_t limit, uint32_t &stepsToFirstInPhase);
static inline void nudgeInsidePhase(Phase p);
static uint32_t measureWidthDense(Phase p, bool forward, uint32_t maxWidth);
FindResult measureCenterToCenter(Phase startPhase, bool forward,
                                 uint32_t searchLimit, uint32_t maxWidth,
                                 uint32_t &centerToCenterSteps,
                                 uint16_t &otherWidthOut);
static inline void bumpToCenter(uint16_t width);
// Verify we're at the expected phase by sampling the sensors a few times
static bool verifyAtPhase(Phase expected, uint8_t samples = 5, uint16_t interSampleMs = 1);

// Wrapper for continuous mode: go to phase, then verify & print result
static FindResult goToPhaseAndVerify(Phase target, bool forward);

// Return true if the sensors consistently match 'expected' over 'samples' reads
static bool verifyAtPhase(Phase expected, uint8_t samples, uint16_t interSampleMs)
{
  uint8_t matches = 0;
  for (uint8_t i = 0; i < samples; ++i)
  {
    if (sensorsMatchPhase(expected))
      matches++;
    if (interSampleMs)
      delay(interSampleMs);
  }
  // majority vote
  return (matches >= (samples / 2 + 1));
}

static FindResult goToPhaseAndVerify(Phase target, bool forward)
{
  FindResult r = goToPhaseAbortable(target, forward);
  if (r != FOUND)
    return r;

  // Verify once on arrival (no extra bumping/motion here)
  bool ok = verifyAtPhase(target, /*samples=*/5, /*interSampleMs=*/1);
  Serial.print("Verify ");
  Serial.print((target == PHASE_FILL) ? "FILL" : "INJECTION");
  Serial.print(": ");
  Serial.println(ok ? "true" : "false");
  return r;
}

// --- Read sensors, return true if they match a phase pattern ---
// Fill:  S1=L, S2=H, S3=L, S4=H
// Inject: S1=H, S2=L, S3=H, S4=H

// Delay for 'ms' while still allowing 'x' abort
static inline void dwellDelayWithAbort(uint32_t ms)
{
  uint32_t start = millis();
  while ((millis() - start) < ms)
  {
    if (pollAbortKey())
      break;
    // light CPU yield
    delay(1);
  }
}

// Flush any pending bytes (e.g., stray CR/LF from the command key)
static inline void flushSerialInput()
{
  while (Serial.available())
    (void)Serial.read();
}

// Read an unsigned integer line from Serial until newline (CR or LF).
// If the user types nothing (or times out), returns defaultVal.
// Bounds are clamped to [minVal, maxVal].
// totalTimeoutMs: overall waiting limit if user never presses Enter
// interCharTimeoutMs: max idle gap between characters after first digit
uint32_t readUintFromSerial(uint32_t defaultVal,
                            uint32_t minVal,
                            uint32_t maxVal,
                            uint32_t totalTimeoutMs = 15000,
                            uint32_t interCharTimeoutMs = 3000)
{
  flushSerialInput(); // clear leftover bytes before prompting

  char buf[16]; // up to 15 digits is plenty
  uint8_t n = 0;
  uint32_t tStart = millis();
  uint32_t tLast = tStart;
  bool anyChar = false;

  // Wait for a line: accumulate digits until CR/LF, or until timeouts
  for (;;)
  {
    // If bytes are available, read them all now
    while (Serial.available())
    {
      char c = Serial.read();
      anyChar = true;
      tLast = millis();

      if (c == '\r' || c == '\n')
      { // end of line
        goto PARSE;
      }
      if (c >= '0' && c <= '9')
      { // keep only digits
        if (n < sizeof(buf) - 1)
          buf[n++] = c;
      }
      // ignore all other chars
    }

    // No bytes right now — check timeouts
    uint32_t now = millis();
    if (!anyChar)
    {
      if (now - tStart >= totalTimeoutMs)
        break; // nothing typed
    }
    else
    {
      if (now - tLast >= interCharTimeoutMs)
        break; // user paused too long
    }
    delay(1);
  }

PARSE:
  buf[n] = 0;

  if (n == 0)
  {
    return defaultVal; // nothing entered
  }

  // Parse the digits we captured
  uint32_t val = 0;
  for (uint8_t i = 0; i < n; ++i)
  {
    val = val * 10u + (uint32_t)(buf[i] - '0');
  }
  if (val < minVal)
    val = minVal;
  if (val > maxVal)
    val = maxVal;
  return val;
}

static inline bool sensorsMatchPhase(Phase p)
{
  int s1 = digitalRead(SENSOR1);
  int s2 = digitalRead(SENSOR2);
  int s3 = digitalRead(SENSOR3);
  int s4 = digitalRead(SENSOR4);

  if (p == PHASE_FILL)
  {
    return (s1 == LOW) && (s2 == HIGH) && (s3 == LOW) && (s4 == HIGH);
  }
  else
  { // PHASE_INJECTION
    return (s1 == HIGH) && (s2 == LOW) && (s3 == HIGH) && (s4 == HIGH);
  }
}

// --- Helpers for sensor monitoring ---
static inline void printSensorStates(uint16_t stepIndex)
{
  int s1 = digitalRead(SENSOR1);
  int s2 = digitalRead(SENSOR2);
  int s3 = digitalRead(SENSOR3);
  int s4 = digitalRead(SENSOR4);

  Serial.print(F("step="));
  Serial.print(stepIndex);
  Serial.print(F(" dir="));
  Serial.print(digitalRead(dir) ? "REV" : "FWD");
  Serial.print(F(" S=["));
  Serial.print(s1 == LOW ? 'L' : 'H');
  Serial.print(' ');
  Serial.print(s2 == LOW ? 'L' : 'H');
  Serial.print(' ');
  Serial.print(s3 == LOW ? 'L' : 'H');
  Serial.print(' ');
  Serial.print(s4 == LOW ? 'L' : 'H');
  Serial.println(']');
}

// --- Core helper: rotate in chosen direction until phase pattern appears ---
void goToPhase(Phase target, bool forward)
{
  // Keep current microstep config; just set direction and enable
  digitalWrite(EN, LOW);
  digitalWrite(dir, forward ? LOW : HIGH);

  const char *dirLabel = forward ? "FWD" : "REV";
  const char *phaseLabel = (target == PHASE_FILL) ? "Hunt FILL" : "Hunt INJECT";
  Serial.print(F("Seeking "));
  Serial.print((target == PHASE_FILL) ? "FILL" : "INJECTION");
  Serial.print(F(" phase "));
  Serial.println(dirLabel);

  // Quick check: already at target?
  if (sensorsMatchPhase(target))
  {
    Serial.println(F("Already at target phase."));
    return;
  }

  uint32_t steps = 0;
  while (steps < MAX_STEPS_TO_FIND)
  {
    // Move in small chunks, then check
    for (uint16_t i = 0; i < CHECK_EVERY; i++)
    {
      stepOnce();
      steps++;
    }
    // printSnapshot(steps, dirLabel, phaseLabel);

    if (sensorsMatchPhase(target))
    {
      Serial.print(F("Reached "));
      Serial.print((target == PHASE_FILL) ? "FILL" : "INJECTION");
      Serial.print(F(" after "));
      Serial.print(steps);
      Serial.println(F(" steps."));
      return;
    }
  }

  Serial.print(F("Failed to find "));
  Serial.print((target == PHASE_FILL) ? "FILL" : "INJECTION");
  Serial.print(F(" within "));
  Serial.print(MAX_STEPS_TO_FIND);
  Serial.println(F(" steps."));
}

// === Add these small wrappers for the four commands ===
void GoToFillForward() { goToPhase(PHASE_FILL, true); }
void GoToFillReverse() { goToPhase(PHASE_FILL, false); }
void GoToInjectForward() { goToPhase(PHASE_INJECTION, true); }
void GoToInjectReverse() { goToPhase(PHASE_INJECTION, false); }

// Optional: print a snapshot when we check
static inline void printSnapshot(uint32_t stepCount, const char *labelDir, const char *labelPhase)
{
  int s1 = digitalRead(SENSOR1);
  int s2 = digitalRead(SENSOR2);
  int s3 = digitalRead(SENSOR3);
  int s4 = digitalRead(SENSOR4);
  Serial.print(labelPhase);
  Serial.print(" ");
  Serial.print(labelDir);
  Serial.print(F(" step="));
  Serial.print(stepCount);
  Serial.print(F(" S=["));
  Serial.print(s1 == LOW ? 'L' : 'H');
  Serial.print(' ');
  Serial.print(s2 == LOW ? 'L' : 'H');
  Serial.print(' ');
  Serial.print(s3 == LOW ? 'L' : 'H');
  Serial.print(' ');
  Serial.print(s4 == LOW ? 'L' : 'H');
  Serial.println(']');
}
// Step N times in current dir (no sensor checks)
static inline void stepN(uint32_t n)
{
  for (uint32_t i = 0; i < n; ++i)
    stepOnce();
}

// From current position, walk until sensors FIRST match 'target'.
// Returns FOUND and writes stepsToFirstInPhase; else NOT_FOUND.
FindResult stepUntilPhaseEnter(Phase target, bool forward, uint32_t limit, uint32_t &stepsToFirstInPhase)
{
  digitalWrite(dir, forward ? LOW : HIGH);
  for (uint32_t i = 0; i < limit; ++i)
  {
    if (sensorsMatchPhase(target))
    {
      stepsToFirstInPhase = i;
      Serial.println("Found target phase.");
      return FOUND;
    }
    stepOnce();
  }
  return NOT_FOUND;
}

// Ensure we are "clearly inside" a phase by taking one more step if we entered on an edge
static inline void nudgeInsidePhase(Phase p)
{
  if (!sensorsMatchPhase(p))
    stepOnce();
}

// Measure phase width densely (1 step/sample) starting while already matching.
// Returns width>=1 else 0 on error.
static uint32_t measureWidthDense(Phase p, bool forward, uint32_t maxWidth)
{
  if (!sensorsMatchPhase(p))
    return 0;
  digitalWrite(dir, forward ? LOW : HIGH);
  uint32_t width = 0;
  for (; width < maxWidth; ++width)
  {
    stepOnce();
    if (!sensorsMatchPhase(p))
      return width;
  }
  return 0;
}

// Compute center→center from a known center of 'startPhase' to center of the opposite phase in 'forward' direction.
// Preconditions: we are at CENTER of startPhase already.
FindResult measureCenterToCenter(Phase startPhase, bool forward,
                                 uint32_t searchLimit, uint32_t maxWidth,
                                 uint32_t &centerToCenterSteps,
                                 uint16_t &otherWidthOut)
{
  Phase target = (startPhase == PHASE_FILL) ? PHASE_INJECTION : PHASE_FILL;

  // 1) Walk until we FIRST enter the target phase
  uint32_t toFirstInPhase = 0;
  FindResult fr = stepUntilPhaseEnter(target, forward, searchLimit, toFirstInPhase);
  if (fr != FOUND)
    return fr;

  // 2) Measure target width densely; center is width/2 from the first in-phase step
  nudgeInsidePhase(target);
  uint32_t width = measureWidthDense(target, forward, maxWidth);
  if (width == 0)
    return NOT_FOUND;

  otherWidthOut = (uint16_t)width;

  // We are one step past the first in-phase step when we call nudge/measure.
  // The center offset from where we STARTED is:
  //   centerToCenter = toFirstInPhase + width/2
  centerToCenterSteps = toFirstInPhase + (width / 2);
  return FOUND;
}

// Convenience: go from a phase edge to its center using a known width
static inline void bumpToCenter(uint16_t width)
{
  // 'width' is measured in full-steps (set during calibration). Scale to the
  // current step mode so this works correctly when called from microstepped
  // continuous mode.
  uint32_t bump = ((uint32_t)width / 2) * microstepMultiplier;
  stepN(bump);
}

// Step once. Pulse width is short (a few us is plenty for the A4988/DRV8825);
// the inter-pulse interval sets the step rate. Full-step needs a longer
// period than microstep to avoid stalling without a ramp.
static inline void stepOnce()
{
  digitalWrite(stp, HIGH);
  delayMicroseconds(3);
  digitalWrite(stp, LOW);
  uint16_t period = (microstepMultiplier > 1) ? stepPeriodUs : FULL_STEP_PERIOD_US;
  delayMicroseconds(period);
}

// Scan in a direction until the sensors first MATCH the phase (edge enter)
// Returns number of steps taken to reach the phase, or 0xFFFFFFFF on failure.
uint32_t seekPhaseEdge(Phase p, bool forward, uint32_t searchLimit)
{
  digitalWrite(dir, forward ? LOW : HIGH);
  for (uint32_t i = 0; i < searchLimit; ++i)
  {
    if (sensorsMatchPhase(p))
      return i; // already inside; i == 0 means we started on the edge/inside
    stepOnce();
  }
  return 0xFFFFFFFFUL; // not found
}

// Once inside a phase, measure its width in steps by stepping until it no longer matches
// Assumes we are ALREADY matching the phase when called. Returns measured width (>=1), else 0 on error.
uint32_t measurePhaseWidth(Phase p, bool forward, uint32_t maxWidth)
{
  // Confirm we are indeed in the phase
  if (!sensorsMatchPhase(p))
    return 0;

  uint32_t width = 0;
  // Count how many consecutive steps remain inside this phase
  for (; width < maxWidth; ++width)
  {
    stepOnce();
    if (!sensorsMatchPhase(p))
    {
      // We just stepped out; the last in-phase step count is 'width'
      return width;
    }
  }
  return 0; // exceeded max width guard
}

// ===== EEPROM persistence for calibration =====
// ===== EEPROM persistence for calibration (v2 adds center→center fields) =====
struct CalData
{
  uint32_t magic;  // validity tag
  uint8_t version; // structure version
  uint8_t reserved[3];

  uint16_t fillWidth;
  uint16_t injectWidth;
  uint16_t fillBump;
  uint16_t injectBump;

  // v2 additions:
  uint16_t stepsF2I; // center->center Fill to Inject (forward)
  uint16_t stepsI2F; // center->center Inject to Fill (reverse)
  uint16_t checksum; // simple sum (excluding this field)
};

static const uint32_t CAL_MAGIC = 0x43514C52UL; // 'CQLR'
static const uint8_t CAL_VERSION = 2;
static const int CAL_EE_ADDR = 0;

static uint16_t calChecksum(const CalData &d)
{
  const uint8_t *p = (const uint8_t *)&d;
  const size_t n = sizeof(CalData) - sizeof(uint16_t); // exclude checksum
  uint16_t s = 0;
  for (size_t i = 0; i < n; ++i)
    s = (uint16_t)(s + p[i]);
  return s;
}

static void calFillFromGlobals(CalData &d)
{
  d.magic = CAL_MAGIC;
  d.version = CAL_VERSION;
  d.reserved[0] = d.reserved[1] = d.reserved[2] = 0;
  d.fillWidth = fillWidthSteps;
  d.injectWidth = injectWidthSteps;
  d.fillBump = fillBumpSteps;
  d.injectBump = injectBumpSteps;
  d.stepsF2I = stepsFillToInject;
  d.stepsI2F = stepsInjectToFill;
  d.checksum = 0;
  d.checksum = calChecksum(d);
}

static bool calLoadToGlobals(const CalData &d)
{
  if (d.magic != CAL_MAGIC)
    return false;
  if (d.version != 1 && d.version != 2)
    return false;
  if (calChecksum(d) != d.checksum)
    return false;

  fillWidthSteps = d.fillWidth;
  injectWidthSteps = d.injectWidth;
  fillBumpSteps = d.fillBump;
  injectBumpSteps = d.injectBump;

  if (d.version >= 2)
  {
    stepsFillToInject = d.stepsF2I;
    stepsInjectToFill = d.stepsI2F;
    calCentersValid = (stepsFillToInject > 0 && stepsInjectToFill > 0);
  }
  else
  {
    stepsFillToInject = stepsInjectToFill = 0;
    calCentersValid = false;
  }
  return true;
}

void SaveCalibrationEEPROM()
{
  CalData d;
  calFillFromGlobals(d);
  EEPROM.put(CAL_EE_ADDR, d);
  Serial.print(F("EEPROM: saved (fillWidth="));
  Serial.print(fillWidthSteps);
  Serial.print(F(", injectWidth="));
  Serial.print(injectWidthSteps);
  Serial.print(F(", fillBump="));
  Serial.print(fillBumpSteps);
  Serial.print(F(", injectBump="));
  Serial.print(injectBumpSteps);
  Serial.println(F(")"));
}

bool LoadCalibrationEEPROM()
{
  CalData d;
  EEPROM.get(CAL_EE_ADDR, d);
  bool ok = calLoadToGlobals(d);
  if (ok)
  {
    Serial.print(F("EEPROM: loaded OK. fillWidth="));
    Serial.print(fillWidthSteps);
    Serial.print(", injectWidth=");
    Serial.print(injectWidthSteps);
    Serial.print(", fillBump=");
    Serial.print(fillBumpSteps);
    Serial.print(", injectBump=");
    Serial.println(injectBumpSteps);
  }
  else
  {
    Serial.println("EEPROM: no valid calibration found (magic/version/checksum fail).");
  }
  return ok;
}

void EraseCalibrationEEPROM()
{
  // Invalidate by clearing magic
  CalData d;
  EEPROM.get(CAL_EE_ADDR, d);
  d.magic = 0xFFFFFFFFUL;
  d.checksum = calChecksum(d);
  EEPROM.put(CAL_EE_ADDR, d);

  // Also zero the in-memory globals so the running session reflects the erase
  // without needing a reboot.
  fillWidthSteps = 0;
  injectWidthSteps = 0;
  fillBumpSteps = 0;
  injectBumpSteps = 0;

  Serial.println("EEPROM: calibration erased (invalidated). In-memory cleared.");
}

void ViewCalibration()
{
  Serial.print("Current calib — fillWidth=");
  Serial.print(fillWidthSteps);
  Serial.print(", injectWidth=");
  Serial.print(injectWidthSteps);
  Serial.print(", fillBump=");
  Serial.print(fillBumpSteps);
  Serial.print(", injectBump=");
  Serial.println(injectBumpSteps);
  Serial.print(", F2I=");
  Serial.print(stepsFillToInject);
  Serial.print(", I2F=");
  Serial.print(stepsInjectToFill);
  Serial.print(", centersValid=");
  Serial.println(calCentersValid ? "Y" : "N");
}

void ContinuousFillInject()
{
  Serial.println("Continuous Fill(REV) <-> Injection(FWD). Press 'x' to stop.");
  Serial.println(F("Live tune: send 'd<ms>' (FILL dwell) or 'j<ms>' (INJECTION dwell), e.g. j250"));
  abortRequested = false;

  // Run continuous mode in 1/8 microstep for smoothness. Step counts (search
  // limit + calibrated bump) are scaled via microstepMultiplier.
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  microstepMultiplier = 8;

  digitalWrite(EN, LOW); // enable

  while (!abortRequested)
  {
    if (goToPhaseAndVerify(PHASE_FILL, /*forward=*/false) == ABORTED)
      break;
    dwellDelayWithAbort(dwellMs);

    if (goToPhaseAndVerify(PHASE_INJECTION, /*forward=*/true) == ABORTED)
      break;
    dwellDelayWithAbort(injectDwellMs);
  }

  digitalWrite(EN, HIGH); // disable

  // Restore full-step so calibration and the F/f/I/i commands behave as before.
  microstepMultiplier = 1;
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);

  Serial.println(F("Continuous mode stopped."));
}

// Non-blocking serial poll for continuous mode.
// Handles:
//   - 'x' / 'X'            -> request abort
//   - "d<digits><term>"    -> live-set FILL dwell (dwellMs)
//   - "j<digits><term>"    -> live-set INJECTION dwell (injectDwellMs)
// where <term> is any non-digit (e.g. newline). Values are clamped to 0..600000 ms.
// Designed for an external (web-serial) controller that streams short ASCII
// commands while the loop is running; parsing is a small state machine so a
// multi-byte number can arrive split across multiple polls.
static inline bool pollAbortKey()
{
  static bool capturing = false;
  static bool targetInject = false;
  static uint32_t acc = 0;
  static bool hasDigits = false;

  while (Serial.available())
  {
    char c = Serial.read();

    if (c == 'x' || c == 'X')
    {
      abortRequested = true;
      capturing = false;
      hasDigits = false;
      acc = 0;
      continue;
    }

    if (!capturing)
    {
      if (c == 'd' || c == 'D' || c == 'j' || c == 'J')
      {
        capturing = true;
        targetInject = (c == 'j' || c == 'J');
        acc = 0;
        hasDigits = false;
      }
      // ignore all other keys during continuous mode
      continue;
    }

    // capturing a value
    if (c >= '0' && c <= '9')
    {
      acc = acc * 10UL + (uint32_t)(c - '0');
      if (acc > 600000UL)
        acc = 600000UL; // clamp to 0..10 minutes
      hasDigits = true;
      continue;
    }

    // Any non-digit terminates the current value
    if (hasDigits)
    {
      if (targetInject)
      {
        injectDwellMs = acc;
        Serial.print(F("injectDwellMs="));
        Serial.println(injectDwellMs);
      }
      else
      {
        dwellMs = acc;
        Serial.print(F("dwellMs="));
        Serial.println(dwellMs);
      }
    }

    // The terminator may itself start the next command.
    if (c == 'd' || c == 'D' || c == 'j' || c == 'J')
    {
      capturing = true;
      targetInject = (c == 'j' || c == 'J');
    }
    else
    {
      capturing = false;
    }
    acc = 0;
    hasDigits = false;
  }
  return abortRequested;
}

// Perform one step and then report sensor states
static inline void stepOnceAndReport(uint16_t idx)
{
  digitalWrite(stp, HIGH);
  delay(2);
  digitalWrite(stp, LOW);
  delay(2);
  printSensorStates(idx);
}

void CalibratePhases()
{
  auto allClose = [](uint16_t a, uint16_t b)
  {
    return (abs((int)a - (int)b) <= CAL_TOL_STEPS);
  };

  Serial.println("=== Calibration: widths + centers + center→center steps (3 trials) ===");
  digitalWrite(EN, LOW);

  uint16_t fillW[CAL_TRIALS] = {0}, injW[CAL_TRIALS] = {0};
  uint16_t f2i[CAL_TRIALS] = {0}, i2f[CAL_TRIALS] = {0};

  for (uint8_t t = 0; t < CAL_TRIALS; ++t)
  {
    Serial.print("[Trial ");
    Serial.print(t + 1);
    Serial.println("]");

    // --- Find FILL edge (reverse), measure width, go to center
    {
      const bool forward = false;
      digitalWrite(dir, HIGH);
      stepN(50); // back off
      uint32_t toEdge = seekPhaseEdge(PHASE_FILL, forward, CALIB_SEARCH_LIMIT);
      if (toEdge == 0xFFFFFFFFUL)
      {
        Serial.println("  ERROR: FILL edge not found");
        goto FAIL;
      }
      if (!sensorsMatchPhase(PHASE_FILL))
        stepOnce();
      uint32_t w = measureWidthDense(PHASE_FILL, forward, CALIB_MAX_WIDTH);
      if (w == 0)
      {
        Serial.println("  ERROR: FILL width failed");
        goto FAIL;
      }
      fillW[t] = (uint16_t)w;

      // After measuring width we stepped out; go back inside and bump to center
      digitalWrite(dir, LOW); // opposite dir to re-enter (LOW is forward per your code)
      stepN(1);
      bumpToCenter(fillW[t]);
    }
    currentPhaseKnown = true;
    currentPhase = PHASE_FILL;

    // --- From FILL center → INJECT center (forward)
    {
      uint32_t stepsC2C = 0;
      uint16_t measureInjW = 0;
      FindResult fr = measureCenterToCenter(PHASE_FILL, /*forward=*/true,
                                            CALIB_SEARCH_LIMIT, CALIB_MAX_WIDTH,
                                            stepsC2C, measureInjW);
      if (fr != FOUND)
      {
        Serial.println("  ERROR: FILL→INJECT measure failed");
        goto FAIL;
      }
      injW[t] = measureInjW;
      f2i[t] = (uint16_t)stepsC2C;

      if (!sensorsMatchPhase(PHASE_INJECTION))
        stepN(1);
      bumpToCenter(injW[t]);
    }
    currentPhaseKnown = true;
    currentPhase = PHASE_INJECTION;

    // --- From INJECT center → FILL center (reverse)
    {
      uint32_t stepsC2C = 0;
      uint16_t measureFillW = 0;
      FindResult fr = measureCenterToCenter(PHASE_INJECTION, /*forward=*/false,
                                            CALIB_SEARCH_LIMIT, CALIB_MAX_WIDTH,
                                            stepsC2C, measureFillW);
      if (fr != FOUND)
      {
        Serial.println("  ERROR: INJECT→FILL measure failed");
        goto FAIL;
      }
      // sanity check inside trial
      if (abs((int)measureFillW - (int)fillW[t]) > CAL_TOL_STEPS)
      {
        Serial.println("  WARN: FILL width varied inside trial; continuing.");
      }
      i2f[t] = (uint16_t)stepsC2C;

      if (!sensorsMatchPhase(PHASE_FILL))
        stepN(1);
      bumpToCenter(fillW[t]);
    }
    currentPhaseKnown = true;
    currentPhase = PHASE_FILL;

    Serial.print("  Trial ");
    Serial.print(t + 1);
    Serial.print(": fillW=");
    Serial.print(fillW[t]);
    Serial.print(", injW=");
    Serial.print(injW[t]);
    Serial.print(", F2I=");
    Serial.print(f2i[t]);
    Serial.print(", I2F=");
    Serial.println(i2f[t]);
  }

  for (uint8_t i = 1; i < CAL_TRIALS; ++i)
  {
    if (!allClose(fillW[i], fillW[0]) || !allClose(injW[i], injW[0]) ||
        !allClose(f2i[i], f2i[0]) || !allClose(i2f[i], i2f[0]))
    {
      Serial.println("ERROR: Calibration trials not consistent (outside tolerance).");
      goto FAIL;
    }
  }

  // --- Commit results ---
  fillWidthSteps = fillW[0];
  injectWidthSteps = injW[0];
  fillBumpSteps = fillWidthSteps / 2;
  injectBumpSteps = injectWidthSteps / 2;
  stepsFillToInject = f2i[0];
  stepsInjectToFill = i2f[0];
  calCentersValid = true;

  Serial.println("=== Calibration complete ===");
  Serial.print("Fill width=");
  Serial.print(fillWidthSteps);
  Serial.print(", Inject width=");
  Serial.print(injectWidthSteps);
  Serial.print(", F2I center→center=");
  Serial.print(stepsFillToInject);
  Serial.print(", I2F center→center=");
  Serial.println(stepsInjectToFill);
  digitalWrite(EN, HIGH);
  return;

FAIL:
  calCentersValid = false;
  Serial.println("Calibration FAILED. Results not saved. Try again.");
  digitalWrite(EN, HIGH);
}

FindResult goToPhaseAbortable(Phase target, bool forward)
{
  // Use calibration if available: center→center open-loop (no constant polling)
  if (calCentersValid)
  {
    bool atFill = sensorsMatchPhase(PHASE_FILL);
    bool atInject = sensorsMatchPhase(PHASE_INJECTION);

    if (atFill)
    {
      currentPhaseKnown = true;
      currentPhase = PHASE_FILL;
    }
    if (atInject)
    {
      currentPhaseKnown = true;
      currentPhase = PHASE_INJECTION;
    }

    if ((target == PHASE_FILL && atFill) || (target == PHASE_INJECTION && atInject))
    {
      // Bump and center→center distances are stored in full-steps; scale to
      // current step mode (continuous mode runs 1/8 microstepped).
      uint16_t bump = (target == PHASE_FILL) ? fillBumpSteps : injectBumpSteps;
      stepN((uint32_t)bump * microstepMultiplier);
      Serial.print("Reached ");
      Serial.println((target == PHASE_FILL) ? "FILL" : "INJECTION");
      return FOUND;
    }

    if (currentPhaseKnown)
    {
      if (currentPhase == PHASE_FILL && target == PHASE_INJECTION)
      {
        digitalWrite(dir, LOW); // forward
        stepN((uint32_t)stepsFillToInject * microstepMultiplier);
        currentPhase = PHASE_INJECTION;
        currentPhaseKnown = true;
        Serial.print("Reached ");
        Serial.println((target == PHASE_FILL) ? "FILL" : "INJECTION");
        return FOUND;
      }
      else if (currentPhase == PHASE_INJECTION && target == PHASE_FILL)
      {
        digitalWrite(dir, HIGH); // reverse
        stepN((uint32_t)stepsInjectToFill * microstepMultiplier);
        currentPhase = PHASE_FILL;
        currentPhaseKnown = true;
        Serial.print("Reached ");
        Serial.println((target == PHASE_FILL) ? "FILL" : "INJECTION");
        return FOUND;
      }
    }

    // One-time quick lock if start phase unknown
    Serial.println("Locking start phase (one-time scan)...");
    uint32_t dummy = 0;
    if (stepUntilPhaseEnter(PHASE_FILL, /*forward=*/false, CALIB_SEARCH_LIMIT, dummy) == FOUND)
    {
      nudgeInsidePhase(PHASE_FILL);
      bumpToCenter(fillWidthSteps);
      Serial.println("Locked to FILL phase.");
      currentPhaseKnown = true;
      currentPhase = PHASE_FILL;
      return goToPhaseAbortable(target, forward);
    }
    if (stepUntilPhaseEnter(PHASE_INJECTION, /*forward=*/true, CALIB_SEARCH_LIMIT, dummy) == FOUND)
    {
      nudgeInsidePhase(PHASE_INJECTION);
      bumpToCenter(injectWidthSteps);
      Serial.println("Locked to INJECTION phase.");
      currentPhaseKnown = true;
      currentPhase = PHASE_INJECTION;
      return goToPhaseAbortable(target, forward);
    }
    Serial.println("Failed to lock start phase quickly; falling back to legacy hunt.");
    // fall through…
  }

  // === Legacy hunt (kept as fallback) ===
  digitalWrite(dir, forward ? LOW : HIGH);

  // Calibrated widths are in full-steps. When running microstepped, scale up.
  const uint32_t maxSteps = (uint32_t)MAX_STEPS_TO_FIND * microstepMultiplier;
  const uint16_t bumpFull = (target == PHASE_FILL) ? fillBumpSteps : injectBumpSteps;
  const uint32_t bumpScaled = (uint32_t)bumpFull * microstepMultiplier;

  if (sensorsMatchPhase(target))
  {
    stepN(bumpScaled);
    currentPhaseKnown = true;
    currentPhase = target;
    return FOUND;
  }

  uint32_t steps = 0;
  while (steps < maxSteps)
  {
    for (uint16_t i = 0; i < CHECK_EVERY; i++)
    {
      stepOnce();
      steps++;
    }
    if (sensorsMatchPhase(target))
    {
      stepN(bumpScaled);
      currentPhaseKnown = true;
      currentPhase = target;
      return FOUND;
    }
    if (pollAbortKey())
    {
      Serial.println("Abort requested.");
      return ABORTED;
    }
  }

  Serial.print("Failed to find target within ");
  Serial.print(maxSteps);
  Serial.println(" steps.");
  return NOT_FOUND;
}

void setup()
{
  // Motor pins
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);

  // Sensor pins with internal pull-ups
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);
  pinMode(SENSOR3, INPUT_PULLUP);
  pinMode(SENSOR4, INPUT_PULLUP);

  resetEDPins();      // default outputs
  Serial.begin(9600); // debug serial

  Serial.println(F("Begin motor control + sensor monitor"));
  Serial.println();
  Serial.println(F("Enter number for control option:"));
  Serial.println(F("1. Turn at default microstep mode (forward)."));
  Serial.println(F("2. Reverse direction at default microstep mode."));
  Serial.println(F("3. Turn at 1/8th microstep mode (forward)."));
  Serial.println(F("4. Turn at 1/8th microstep mode (reverse)."));
  Serial.println(F("F. Go to FILL (forward)"));
  Serial.println(F("f. Go to FILL (reverse)"));
  Serial.println(F("I. Go to INJECTION (forward)"));
  Serial.println(F("i. Go to INJECTION (reverse)"));
  Serial.println(F("C. Continuous FILL (rev) <-> INJECTION (fwd). Press 'x' to stop."));
  Serial.println(F("K. Calibrate widths + centers + center→center (3 trials)."));
  Serial.println(F("S. Save calibration to EEPROM"));
  Serial.println(F("L. Load calibration from EEPROM"));
  Serial.println(F("E. Erase calibration in EEPROM"));
  Serial.println(F("V. View current calibration"));
  Serial.println(F("D. Set continuous-mode FILL dwell (ms)"));
  Serial.println(F("J. Set continuous-mode INJECTION dwell (ms)"));
  Serial.println(F("P. Set step period (us) — lower = faster motion"));

  if (!LoadCalibrationEEPROM())
  {
    Serial.println(F("Tip: run 'K' (calibrate) then 'S' (save) to store to EEPROM."));
  }

  Serial.println();
}

void loop()
{
  while (Serial.available())
  {
    user_input = Serial.read(); // Read user input and trigger appropriate function
    digitalWrite(EN, LOW);      // Enable driver (active low)

    if (user_input == '1')
    {
      StepForwardDefault();
    }
    else if (user_input == '2')
    {
      ReverseStepDefault();
    }
    else if (user_input == '3')
    {
      SmallStepModeForward();
    }
    else if (user_input == '4')
    {
      SmallStepModeReverse();
    }
    else if (user_input == 'F')
    {
      GoToFillForward();
    }
    else if (user_input == 'f')
    {
      GoToFillReverse();
    }
    else if (user_input == 'I')
    {
      GoToInjectForward();
    }
    else if (user_input == 'i')
    {
      GoToInjectReverse();
    }
    else if (user_input == 'C' || user_input == 'c')
    {
      ContinuousFillInject();
    }
    else if (user_input == 'K' || user_input == 'k')
    {
      CalibratePhases();
    }
    else if (user_input == 'S')
    {
      SaveCalibrationEEPROM();
    }
    else if (user_input == 'L')
    {
      LoadCalibrationEEPROM();
    }
    else if (user_input == 'E')
    {
      EraseCalibrationEEPROM();
    }
    else if (user_input == 'V')
    {
      ViewCalibration();
      Serial.print(F(", dwellMs="));
      Serial.print(dwellMs);
      Serial.print(F(", injectDwellMs="));
      Serial.print(injectDwellMs);
      Serial.print(F(", stepPeriodUs="));
      Serial.println(stepPeriodUs);
      Serial.println(F("Use 'D'/'J' to change FILL/INJECTION dwell, 'P' to change step period."));
    }
    else if (user_input == 'D' || user_input == 'd')
    {
      Serial.print(F("Enter FILL dwell ms (current="));
      Serial.print(dwellMs);
      Serial.println(F("):"));
      uint32_t val = readUintFromSerial(dwellMs, 0, 600000UL); // 0..10 minutes
      dwellMs = val;
      Serial.print(F("FILL dwell set to "));
      Serial.print(dwellMs);
      Serial.println(F(" ms"));
    }
    else if (user_input == 'J' || user_input == 'j')
    {
      Serial.print(F("Enter INJECTION dwell ms (current="));
      Serial.print(injectDwellMs);
      Serial.println(F("):"));
      uint32_t val = readUintFromSerial(injectDwellMs, 0, 600000UL); // 0..10 minutes
      injectDwellMs = val;
      Serial.print(F("INJECTION dwell set to "));
      Serial.print(injectDwellMs);
      Serial.println(F(" ms"));
    }
    else if (user_input == 'P' || user_input == 'p')
    {
      Serial.print(F("Enter step period us (current="));
      Serial.print(stepPeriodUs);
      Serial.println(F("). Smaller = faster. Try 300-1500."));
      // Clamp: too short risks missed steps / driver violation; too long is pointless.
      uint32_t val = readUintFromSerial(stepPeriodUs, 100, 10000UL);
      stepPeriodUs = (uint16_t)val;
      Serial.print(F("Step period set to "));
      Serial.print(stepPeriodUs);
      Serial.println(F(" us"));
    }
    else
    {
      Serial.println("Invalid option entered.");
    }
    resetEDPins(); // disable after action
  }
}

// ==== Motion functions (now call stepOnceAndReport) ====

static inline void stepOnceAndMaybeReport(uint16_t idx)
{
  digitalWrite(stp, HIGH);
  delay(1);
  digitalWrite(stp, LOW);
  delay(1);
  if ((idx % REPORT_EVERY) == 0)
  {
    printSensorStates(idx);
  }
}

// ==== In your motion functions, replace stepOnceAndReport(x) with: ====

void StepForwardDefault()
{
  Serial.println("Moving forward at default step mode.");
  digitalWrite(dir, LOW);
  for (x = 0; x < 100; x++)
  {
    stepOnceAndMaybeReport(x);
  }
  Serial.println("Enter new option\n");
}

void ReverseStepDefault()
{
  Serial.println("Moving in reverse at default step mode.");
  digitalWrite(dir, HIGH);
  for (x = 0; x < 100; x++)
  {
    stepOnceAndMaybeReport(x);
  }
  Serial.println("Enter new option\n");
}

void SmallStepModeForward()
{
  Serial.println("Stepping forward at 1/8th microstep mode.");
  digitalWrite(dir, LOW);
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  for (x = 0; x < 10; x++)
  {
    stepOnceAndMaybeReport(x);
  }
  Serial.println("Enter new option\n");
}

void SmallStepModeReverse()
{
  Serial.println("Stepping reverse at 1/8th microstep mode.");
  digitalWrite(dir, HIGH);
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  for (x = 0; x < 10; x++)
  {
    stepOnceAndMaybeReport(x);
  }
  Serial.println("Enter new option\n");
}

void ForwardBackwardStep()
{
  Serial.println("Alternate between stepping forward and reverse.");
  for (x = 1; x < 5; x++)
  {
    state = digitalRead(dir);
    digitalWrite(dir, state == HIGH ? LOW : HIGH);

    for (y = 0; y < 1000; y++)
    {
      stepOnceAndReport(y);
    }
  }
  Serial.println("Enter new option:");
  Serial.println();
}

// Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN, HIGH); // disable driver
}
