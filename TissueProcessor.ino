#define DEBUG_SERIAL // Uncomment to enable Serial output for debugging
#define TEST_MODE    // Uncomment to enable fast timers for testing

// ===================== DEBUG MACROS =====================
#ifdef DEBUG_SERIAL
#define DBG(x) Serial.print(x)
#define DBGLN(x) Serial.println(x)
#else
#define DBG(x)
#define DBGLN(x)
#endif

/*
  Wax Bath / Tissue Processor Controller
  - Implements the sequential logic described by the user.
  - Uses the MicroBeaut Finite-State library for structured state transitions.
  - All comments in English.

  Pin mapping (as requested):
    Power outputs: D9..D12
      D12 -> vibration motor (VIB_PIN)
      D11 -> movement motor (MOVE_PIN)
      D10 -> heater 1 (HEATER1_PIN)
      D9  -> heater 2 (HEATER2_PIN)

    Sensor inputs (active LOW): D5..D8
      D8 -> top-level sensor (TOP_SENSOR_PIN)
      D7 -> bottom-level sensor (BOTTOM_SENSOR_PIN)
      D6 -> wax-ready sensor (WAX1_SENSOR_PIN)
      D5 -> wax-ready sensor 2 (WAX2_SENSOR_PIN)

    Tank selector: A0..A3 (4-bit binary, normal analogRead interpreted as digital 0/1)
    Start/Smart button: D2
    I2C LCD: A4 (SDA), A5 (SCL)

  Notes / safety:
    - Sensors are treated active when digitalRead == LOW.
    - Mechanical movement timeout: 30 seconds (if motor runs without reaching target sensor).
    - 1 second delay when switching between vibration and movement motors.
    - Fail-safe: Any sensor reading LOW unexpectedly stops motion and sets ERROR state.

  Libraries required:
    - Finite-State (https://github.com/MicroBeaut/Finite-State)
    - LiquidCrystal_I2C (for I2C LCD, typical address 0x27)
*/

#include <FiniteState.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ========================= PIN MAP (as provided) =========================
// Power outputs
const uint8_t VIB_PIN = 9;      // D9  - Heater 2 (power unit)
const uint8_t MOVE_PIN = 10;    // D10 - Heater 1
const uint8_t HEATER1_PIN = 11; // D11 - Relay enable for AC motor
const uint8_t HEATER2_PIN = 12; // D12 - Processing vibration motor
// Sesnors &  thermostats (Active LOW: 0 = active)
const uint8_t SENSOR_WAX2 = 5;   // D5
const uint8_t SENSOR_WAX1 = 6;   // D6
const uint8_t SENSOR_BOTTOM = 7; // D7
const uint8_t SENSOR_TOP = 8;    // D8

// Container ID (A0..A3)
const uint8_t PIN_ID_BITS[4] = {A0, A1, A2, A3}; // bit0 LSB .. bit3 MSB

// UI
const uint8_t START_BUTTON = 2; // D2

// LCD (I2C)
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ========================= CONFIGURATION & TIME CONSTANTS =========================
#ifdef TEST_MODE
const unsigned long ONE_MIN_MS = 1000UL; // 1 second = 1 "minute" for testing
const unsigned long MIN_DWELL_MIN = 1UL; // allow 1 "minute" in test
#else
const unsigned long ONE_MIN_MS = 60UL * 1000UL; // 1 real minute
const unsigned long MIN_DWELL_MIN = 60UL;       // production min dwell in minutes
#endif

const unsigned long MOVE_TIMEOUT_MS = 30UL * 1000UL;          // 30 seconds - motion safety timeout
const unsigned long DEBOUNCE_MS = 40UL;                       // button debounce
const unsigned long LCD_REFRESH_MS = 1000UL;                  // 1s LCD refresh
const unsigned long FAULT_RESET_HOLD_MS = 5000UL;             // hold RUN 5s to clear fault
const unsigned long MOTOR_SWITCH_DELAY_MS = 1000UL;           // 1 second - motor swithc saf
const unsigned long CONTAINER_TIME_MS = 60UL * 60UL * 1000UL; // Normaly stays down for 1 hour while vibrating
const unsigned long START_BUTTON_DELAY_MS = 2UL * 1000UL;     // Idle state - 2 seconds

// Relay settle for AC motor (ms)
const unsigned long RELAY_SETTLE_MS = 20UL; // 15-30 ms recommended

// Constants derived from dwellMinutes
const unsigned long TRANSITION_DELAY_MS = 2000UL; // short transition between tanks

// Per-container dwell times in minutes (index 1..12)
#ifdef TEST_MODE
unsigned int dwellMinutes[13] = {
    0,                         // 0 unused
    1, 1, 1, 1, 1, 1, 1, 1, 1, // containers 1..9
    1,                         // 10
    2,                         // 11
    2                          // 12
};
#else
unsigned int dwellMinutes[13] = {
    0,
    60, 60, 60, 60, 60, 60, 60, 60, 60, // containers 1..9
    60,                                 // 10
    120,                                // 11
    120                                 // 12
};
#endif

// Program variables
volatile bool startButtonPressed = false; // updated in ISR (debounced in FSM logic)

uint8_t currentTank = 1;            // 1..12
unsigned long startTimeTank = 0;    // for time in tank
unsigned long processStartTime = 0; // for per-tank timing
unsigned long motorStartTime = 0;   // for mechanical timeout
unsigned long lastMotorOffTime = 0; // for motor switch safety
unsigned long pressedAt = 0;        // for press button
bool lastButton = LOW;              // last button state
bool finished = false;              // Cycle finished
bool inspection = false;            // Inspection activated
bool firstCycle11 = false;          // First cycle completed on 11th container
bool firstCycle12 = false;          // First cycle completed on 12th container
bool holdHandled = false;           // If we processed the button

// Utility: read tank number from A0..A3 as digital inputs (0..15) then +1 (1..16) clamp to 1..12
uint8_t readTankSelector()
{
  uint8_t val = 0;
  for (uint8_t i = 0; i < 4; ++i)
  {
    val |= (digitalRead(PIN_ID_BITS[i]) == HIGH) << i;
  }
  uint8_t tank = val + 1;
  if (tank < 1)
    tank = 1;
  if (tank > 12)
    tank = 12;
  return tank;
}

// Sensor helpers (active LOW). Return true when sensor is active (LOW) or when a fail-safe (treated as active).
bool sensorActive(uint8_t pin)
{
  // We intentionally DO NOT enable INPUT_PULLUP here for external wiring; a broken wire should report LOW in the user's spec.
  return digitalRead(pin) == LOW;
}

// Motor control helpers
void vibOn()
{
  digitalWrite(VIB_PIN, HIGH);
  DBGLN("VIB ON");
}
void vibOff()
{
  digitalWrite(VIB_PIN, LOW);
  DBGLN("VIB OFF");
}
void moveOn()
{
  digitalWrite(MOVE_PIN, HIGH);
  motorStartTime = millis();
  DBGLN("MOVE ON");
}
void moveOff()
{
  digitalWrite(MOVE_PIN, LOW);
  motorStartTime = 0;
  lastMotorOffTime = millis();
  DBGLN("MOVE OFF");
}

// Display helper
void lcdShowStatus(const char *line1, const char *line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// Print remaining time for current tank
void printRemainingTimeForTank(uint8_t tank)
{
  unsigned int mins = dwellMinutes[tank];

  // 1. Safety fallback
  if (mins == 0)
    mins = MIN_DWELL_MIN;

  // 2. Adjust for specific tank logic
  if (tank == 11 && firstCycle11)
    mins -= MIN_DWELL_MIN;
  if (tank == 12 && firstCycle12)
    mins -= MIN_DWELL_MIN;

  // 3. Calculate Remaining Time
  // Assuming 'startTimeTank' was set when the process began:
  unsigned long totalMs = (unsigned long)mins * ONE_MIN_MS;
  unsigned long elapsedMs = millis() - startTimeTank;

  long remainingMs = (long)totalMs - (long)elapsedMs;

  // Prevent negative numbers if the timer has expired
  if (remainingMs < 0)
    remainingMs = 0;

  // 4. Convert to Hours and Minutes
  unsigned int remainingTotalMins = remainingMs / 60000;
  unsigned int hours = remainingTotalMins / 60;
  unsigned int displayMins = remainingTotalMins % 60;

  // 5. Debug Printing
  DBG("Remaining Time: 0");
  DBG(hours);
  DBG(":");
  if (displayMins < 10)
    DBG("0"); // Leading zero for minutes
  DBGLN(displayMins);
}

/**
 * Returns true only after the button has been held for 'duration'
 */
bool buttonHeld(uint8_t button, uint32_t duration)
{
  uint8_t state = digitalRead(button); // Assumes LOW when pressed (INPUT_PULLUP)

  if (state == LOW)
  {
    if (pressedAt == 0)
    {
      // Just started pressing
      pressedAt = millis();
      holdHandled = false;
    }
    else if (!holdHandled && (millis() - pressedAt >= duration))
    {
      // Threshold reached!
      holdHandled = true;
      return true;
    }
  }
  else
  {
    // Button released - reset everything
    pressedAt = 0;
    holdHandled = false;
  }

  return false;
}

// FSM state enumeration
enum MainState : id_t
{
  S_IDLE = 0,
  S_START,
  S_LOWER,
  S_DOWN,
  S_CHECK_STATUS,
  S_RAISE,
  S_TOP,
  S_TRANSITION,
  S_ERROR
};

// Forward declarations for predicate/process/event functions
bool idlePredicate(id_t d);
void idleProcess(id_t id);
void IdleActionChanged(EventArgs e);

void startProcess(id_t id);
bool startPredicate(id_t id);
void StartActionChanged(EventArgs e);

void lowerProcess(id_t id);
bool lowerPredicate(id_t id);
void lowerActionChanged(EventArgs e);

void downProcess(id_t id);
bool downPredicate(id_t id);
void downActionChanged(EventArgs e);

void topProcess(id_t id);
bool topPredicate(id_t id);
void topActionChanged(EventArgs e);

void raiseProcess(id_t id);
bool raisePredicate(id_t id);
void raiseActionChanged(EventArgs e);

void checkStatusProcess(id_t id);
bool checkStatusPredicate(id_t id);
void checkStatusActionChanged(EventArgs e);

void transitionProcess(id_t id);
bool transitionPredicate(id_t id);
void transitionActionChanged(EventArgs e);

void errorProcess(id_t id);
bool errorPredicate(id_t id);
void errorActionChanged(EventArgs e);

// Transition table - keep it as readable blocks. Use predicate timers where necessary.
Transition transitions[] = {
    // S_IDLE: wait start button. When pressed -> CHECK_START
    {idlePredicate, S_IDLE, S_START, idleProcess, IdleActionChanged},

    // S_START: read tank and prepare to start.
    {startPredicate, S_IDLE, S_LOWER, startProcess, StartActionChanged},

    // S_LOWER: run movement motor until bottom sensor active OR timeout -> if bottom sensor active -> VIBRATE else ERROR
    {lowerPredicate, S_LOWER, S_DOWN, lowerProcess, lowerActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    /*S_DOWN: Vibrating for 1 hour
              Conatiner 10 -> Start first heater.
              Container 11 -> Start second heater.
              Container 12 -> If finished then stop vibrating.
    */
    {downPredicate, S_RAISE, S_CHECK_STATUS, downProcess, downActionChanged, CONTAINER_TIME_MS, TRUE_TIMER},

    /*S_CHECK_STATUS: Container 1..10 -> continue to raise state
              Conatiner 11 + 12 -> Two hours instead of 1 hour, so renter the down state.
              Conatiner 10 -> Renter if first wax sensor is not ready.
              Conatiner 11 -> Renter if first wax sensor is not ready.

    */
    {checkStatusPredicate, S_DOWN, S_RAISE, checkStatusProcess, checkStatusActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER}

    // S_RAISE: run movement up until top sensor active OR timeout -> TOP or ERROR
    {raisePredicate, S_RAISE, S_TOP, raiseProcess, raiseActionChanged},

    // S_TOP: wait if insepction is acitvitated, otherwise proceed to transition state
    {topPredicate, S_TOP, S_TRANSITION, topProcess, topActionChanged},

    // S_TRANSITION: small delay between tanks, then either go to next tank's logic or to START if finished
    {transitionPredicate, S_TRANSITION, S_START, transitionProcess, transitionActionChanged},

    // ERROR
    {errorPredicate, S_ERROR, S_ERROR, errorProcess, errorActionChanged}};

const uint8_t numberOfTransitions = sizeof(transitions) / sizeof(Transition);

FiniteState fsm(transitions, numberOfTransitions);

// Implementation: Predicates, Processes and Events
bool idlePredicate(id_t d)
{
  if (buttonHeld(START_BUTTON, START_BUTTON_DELAY_MS))
  {
    DBGLN("Start button pressed");
    return true;
  }
  return false;
}
void idleProcess(id_t id)
{
  // display waiting state
  lcdShowStatus("Status: Idle", "Press Start");
  return;
}

void IdleActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Enter idle");
    finished = false;
    firstCycle11 = false;
    firstCycle12 = false;
    break;
  case EXIT:
    DBGLN("Exit idle");
    break;
  }
}
void IdleActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Enter idle");
    finished = false;
    firstCycle11 = false;
    firstCycle12 = false;
    break;
  case EXIT:
    DBGLN("Exit idle");
    break;
  }
}

bool startPredicate(id_t id)
{
  if (!sensorActive(SENSOR_TOP))
    return false; // Top sensor is not active -> Error

  uint8_t tankId = readTankSelector;

  if (tankId == 11 && !sensorActive(HEATER1_PIN))
    return false;

  if (tankId == 12 && !sensorActive(HEATER2_PIN))
    return false;

  if (finished)
    return false; // Finished cycle -> false

  return true;
}

void eventOnActionChanged(EventArgs e)
{
  // handle entry/exit actions
  switch (e.action)
  {
  case ENTRY:
    // when entering a state, show the state on LCD
    switch (e.id)
    {
    case S_START:
      currentTank = readTankSelector();
      {
        char buf1[17];
        char buf2[17];
        snprintf(buf1, sizeof(buf1), "Tank: %u", currentTank);
        snprintf(buf2, sizeof(buf2), "Starting...");
        lcdShowStatus(buf1, buf2);
        DBG("Entering CHECK_START for tank: ");
        DBGLN(currentTank);
      }
      break;

    case S_LOWER:
      lcdShowStatus("Lowering", "Waiting bottom");
      DBGLN("State: LOWER");
      break;

    case S_VIBRATE:
      lcdShowStatus("Processing", "Vibration ON");
      DBGLN("State: VIBRATE");
      break;

    case S_DOWN:
      lcdShowStatus("Processing", "Timer running");
      DBGLN("State: WAIT_PROCESS");
      break;

    case S_STOP_VIBRATE:
      lcdShowStatus("Processing", "Stopping vibration");
      break;

    case S_RAISE:
      lcdShowStatus("Raising", "Waiting top");
      break;

    case S_TRANSITION:
      lcdShowStatus("Transition", "Moving tank");
      break;

    case S_WAX_TANK_10_HEAT:
      lcdShowStatus("Tank 10", "Heating wax");
      break;

    case S_WAX_TANK_10_WAIT:
      lcdShowStatus("Tank 10", "Waiting wax melt");
      break;

    case S_WAX_TANK_11_HEAT:
      lcdShowStatus("Tank 11", "Heating wax");
      break;

    case S_WAX_TANK_11_WAIT:
      lcdShowStatus("Tank 11", "Waiting wax melt");
      break;

    case S_COMPLETE:
      lcdShowStatus("Cycle Complete", "All off");
      break;

    case S_PAUSED:
      lcdShowStatus("Paused", "Press Start");
      break;

    case S_ERROR:
      lcdShowStatus("ERROR", "Check sensors");
      break;

    default:
      break;
    }
    break;

  case EXIT:
    // nothing in general
    break;

  default:
    break;
  }
}

// LOWER state: start movement motor and watch for bottom sensor or timeout
void lowerProcess(id_t id)
{
  // start movement motor if safe to do so
  // ensure we have waited 1s after stopping other motor
  if (millis() - lastMotorOffTime < MOTOR_SWITCH_DELAY_MS)
  {
    moveOff();
    return;
  }
  moveOn();
  // check mechanical timeout
  if (motorStartTime == 0)
    motorStartTime = millis();
}

bool lowerPredicate(id_t id)
{
  // if bottom sensor active -> true to move to VIBRATE
  if (sensorActive(SENSOR_BOTTOM))
  {
    moveOff();
    motorStartTime = 0;
    DBGLN("Reached bottom");
    return true;
  }
  // mechanical timeout
  if (motorStartTime && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
  {
    // timeout -> error
    moveOff();
    DBGLN("Lower timeout -> ERROR");
    fsm.begin(S_ERROR);
    return false; // remain or exit to error as FSM sets state elsewhere
  }
  // fail-safe: if any critical sensor reads active unexpectedly -> error
  // (e.g., top sensor triggered while lowering)
  if (sensorActive(SENSOR_TOP) && !sensorActive(SENSOR_BOTTOM))
  {
    moveOff();
    DBGLN("Top triggered while lowering -> ERROR");
    fsm.begin(S_ERROR);
  }
  return false; // continue lowering
}

// VIBRATE: enable vibration and start timer
void downProcess(id_t id)
{
  // Ensure motor switch safety
  if (millis() - lastMotorOffTime < MOTOR_SWITCH_DELAY_MS)
  {
    vibOff();
    return;
  }
  vibOn();
  processStartTime = millis();
}

bool downPredicate(id_t id)
{
  // Immediately transition to WAIT_PROCESS after starting vibration
  return true;
}

// WAIT_PROCESS: main countdown using printRemainingTimeForTank
void WaitProcess(id_t id)
{
  // Keep vibration on while waiting, check special cases for wax tanks
  vibOn();
  unsigned long desired = printRemainingTimeForTank(currentTank);

  // For tank 10 special rule: stay in bottom area with vibration on until WAX1 sensor becomes active
  if (currentTank == 10)
  {
    if (sensorActive(SENSOR_WAX1))
    {
      if (processStartTime == 0)
        processStartTime = millis();
    }
    else
    {
      // still waiting for wax melt: ensure movement motor off and vibration on
      moveOff();
      vibOn();
      lcdShowStatus("Tank10", "Waiting wax melt");
      DBGLN("Tank10: waiting for wax melt");
      return;
    }
  }

  // For tank 11 special: require dwellMinutes[11]
  if (currentTank == 11)
  {
    desired = printRemainingTimeForTank(11);
  }

  if (processStartTime == 0)
    processStartTime = millis();

  // safety check
  if (sensorActive(SENSOR_TOP) && sensorActive(SENSOR_BOTTOM))
  {
    DBGLN("Both sensors active -> ERROR");
    fsm.begin(S_ERROR);
    return;
  }

  if (millis() - processStartTime >= desired)
  {
    processStartTime = 0;
    DBGLN("Processing time complete for tank");
    // FSM will consult WaitPredicate to transition
  }
}

bool WaitPredicate(id_t id)
{
  unsigned long desired = printRemainingTimeForTank(currentTank);

  if (currentTank == 10)
  {
    static unsigned long meltDetectedAt = 0;
    if (sensorActive(SENSOR_WAX1))
    {
      if (meltDetectedAt == 0)
        meltDetectedAt = millis();
      if (millis() - meltDetectedAt >= desired)
      {
        meltDetectedAt = 0;
        DBGLN("Tank10: melt+time satisfied");
        return true;
      }
    }
    else
    {
      meltDetectedAt = 0;
      return false;
    }
    return false;
  }

  if (processStartTime == 0)
    return false;
  if (millis() - processStartTime >= desired)
    return true;
  return false;
}

void stopVibrateProcess(id_t id)
{
  vibOff();
  // ensure motor is off before switching
  moveOff();
}

bool stopVibratePredicate(id_t id)
{
  if (id == S_)
}

// RAISE: turn on movement motor to bring to top sensor
void raiseProcess(id_t id)
{
  // respect motor switch delay
  if (millis() - lastMotorOffTime < MOTOR_SWITCH_DELAY_MS)
  {
    moveOff();
    return;
  }
  moveOn();
  if (motorStartTime == 0)
    motorStartTime = millis();
}

bool raisePredicate(id_t id)
{
  if (sensorActive(SENSOR_TOP))
  {
    motorStartTime = 0;
    DBGLN("Reached top");
    return true;
  }
  if (motorStartTime && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
  {
    // timeout
    moveOff();
    DBGLN("Raise timeout -> ERROR");
    fsm.begin(S_ERROR);
    return false;
  }
  return false;
}

void transitionProcess(id_t id)
{
  DBGLN("transitionProcess: increment tank");
  if (currentTank >= 12)
  {
    // cycle complete: shutdown everything
    heater1Off();
    heater2Off();
    vibOff();
    moveOff();
    fsm.begin(S_COMPLETE);
    return;
  }
  currentTank++;

  // Decide if next tank is wax-specific start states
  if (currentTank == 10)
  {
    fsm.begin(S_WAX_TANK_10_HEAT);
    return;
  }
  if (currentTank == 11)
  {
    fsm.begin(S_WAX_TANK_11_HEAT);
    return;
  }

  // default: go to LOWER for next tank
  fsm.begin(S_LOWER);
}

// Wax tank 10 heating: enable heater1 and wait for wax sensor
void Wax10HeatProcess(id_t id)
{
  heater1On();
  vibOn();
  moveOff();
}

if (sensorActive(SENSOR_WAX1))
{
  bool Wax10HeatPredicate(id_t id)
  {
    DBGLN("Wax10: melt detected");
    return true;
  }
  return false;
}

void Wax10WaitProcess(id_t id)
{
  unsigned long desired = printRemainingTimeForTank(10);
  if (processStartTime == 0)
    processStartTime = millis();
  vibOn();
  if (millis() - processStartTime >= desired)
  {
    processStartTime = 0;
    heater1Off();
    fsm.begin(S_STOP_VIBRATE);
  }
}

bool Wax10WaitPredicate(id_t id)
{
  return false;
}

// Wax tank 11 heating: enable heater2 and ensure wax melted before descending; if not melted -> error
void Wax11HeatProcess(id_t id)
{
  heater2On();
  moveOff();
}

bool Wax11HeatPredicate(id_t id)
{
  if (!sensorActive(SENSOR_WAX1))
  {
    lcdShowStatus("Error", "Prev wax not melted");
    DBGLN("Wax11: prev wax not melted -> ERROR");
    fsm.begin(S_ERROR);
    return false;
  }
  if (sensorActive(SENSOR_WAX2))
  {
    DBGLN("Wax11: melt detected");
    return true;
  }
  return false;
}

void Wax11WaitProcess(id_t id)
{
  vibOn();
  if (processStartTime == 0)
    processStartTime = millis();
  unsigned long desired = printRemainingTimeForTank(11);
  if (millis() - processStartTime >= desired)
  {
    processStartTime = 0;
    heater2Off();
    fsm.begin(S_STOP_VIBRATE);
  }
}

bool Wax11WaitPredicate(id_t id)
{
  return false;
}

// PAUSE process - used when smart button pressed while tank is down
void PauseProcess(id_t id)
{
  moveOff();
  vibOff();
}

void errorProcess(id_t id)
{
  moveOff();
  vibOff();
  heater1Off();
  heater2Off();
  lcdShowStatus("ERROR", "Manual reset req");
  DBGLN("Entered ERROR state");
}

// Setup and loop

void setupPins()
{
  pinMode(VIB_PIN, OUTPUT);
  digitalWrite(VIB_PIN, LOW);
  pinMode(MOVE_PIN, OUTPUT);
  digitalWrite(MOVE_PIN, LOW);
  pinMode(HEATER1_PIN, OUTPUT);
  digitalWrite(HEATER1_PIN, LOW);
  pinMode(HEATER2_PIN, OUTPUT);
  digitalWrite(HEATER2_PIN, LOW);

  // sensor inputs - active LOW; do not enable pullups to match fail-safe behavior described
  pinMode(SENSOR_WAX2, INPUT);
  pinMode(SENSOR_WAX1, INPUT);
  pinMode(SENSOR_BOTTOM, INPUT);
  pinMode(SENSOR_TOP, INPUT);

  pinMode(START_BUTTON, INPUT_PULLUP); // use internal pullup and interpret LOW as pressed

  for (uint8_t i = 0; i < 4; ++i)
    pinMode(PIN_ID_BITS[i], INPUT_PULLUP);
}

void setup()
{
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  while (!Serial)
    ;
  DBGLN("Serial debug enabled");
#endif
  setupPins();
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // start FSM in IDLE
  fsm.begin(S_IDLE);
}

void loop()
{
  // Execute FSM regularly
  fsm.execute();

  // Monitor start button behavior for pause/advance logic (smart button)
  bool curr = digitalRead(START_BUTTON);

  if (curr == HIGH)
  {
    if (pressedAt == 0)
      pressedAt = millis();
  }
  else
  {
    pressedAt = 0;
  }

  if (curr == HIGH && millis() - pressedAt > FAULT_RESET_HOLD_MS)
  {
    DBGLN("Run button pressed");
    if (sensorActive(SENSOR_BOTTOM))
    {
      if (fsm.id != S_PAUSED)
      {
        fsm.begin(S_PAUSED);
      }
      else if (sensorActive(SENSOR_TOP))
      {
        fsm.begin(S_TRANSITION);
      }
    }
    else
    {
      fsm.begin(S_TRANSITION);
    }
  }
  lastButton = curr;

  // Small watchdog: if FSM in error state, require manual reset via long press of start button
  if (fsm.id == S_ERROR && millis() - pressedAt > FAULT_RESET_HOLD_MS)
  {
    pressedAt = 0;
    fsm.begin(S_IDLE);
  }

  delay(10);
}
