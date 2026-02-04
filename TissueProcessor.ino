#define DEBUG // Uncomment to enable Serial output for debugging
#define TEST  // Uncomment to enable fast timers for testing

// ===================== DEBUG MACROS =====================
#ifdef DEBUG
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

  Pin mapping:
    Power outputs: D9..D12
      D11 -> movement motor (MOVE_PIN)
      D12 -> vibration motor (VIB_PIN)
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
    - Finite-State 1.6.0 (https://github.com/MicroBeaut/Finite-State) added locally...
    - LiquidCrystal_I2C (for I2C LCD, typical address 0x27)
*/

#include <FiniteState.h> // 1.6.0
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // 1.1.2

// ========================= PIN MAP (as provided) =========================
// Power outputs
const uint8_t VIB_PIN = 11;     // D11 - Relay enable for AC motor
const uint8_t MOVE_PIN = 12;    // D12 - Processing vibration motor
const uint8_t HEATER1_PIN = 10; // D10  - Heater 2 (power unit)
const uint8_t HEATER2_PIN = 9;  // D9   - Heater 1
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
#ifdef TEST
const unsigned long ONE_MIN_MS = 1000UL; // 1 second = 1 "minute" for testing
const unsigned long MIN_DWELL_MIN = 1UL; // allow 1 "minute" in test
#else
const unsigned long ONE_MIN_MS = 60UL * 1000UL; // 1 real minute
const unsigned long MIN_DWELL_MIN = 60UL;       // production min dwell in minutes
#endif

const unsigned long MOVE_TIMEOUT_MS = 30UL * 1000UL;          // 30 seconds - motion safety timeout
const unsigned long MOTOR_SWITCH_DELAY_MS = 1000UL;           // 1 second - motor swithc saf
const unsigned long CONTAINER_TIME_MS = 60UL * 60UL * 1000UL; // Normaly stays down for 1 hour while vibrating
const unsigned long START_BUTTON_DELAY_MS = 2UL * 1000UL;     // Idle state - 2 seconds

// Constants derived from dwellMinutes
const unsigned long TRANSITION_DELAY_MS = 2000UL; // short transition between tanks

// Per-tank dwell times in minutes (index 1..12)
#ifdef TEST
const unsigned int dwellMinutes[13] = {
    0,                         // 0 unused
    1, 1, 1, 1, 1, 1, 1, 1, 1, // containers 1..9
    1,                         // 10
    2,                         // 11
    2                          // 12
};
#else
const unsigned int dwellMinutes[13] = {
    0,
    60, 60, 60, 60, 60, 60, 60, 60, 60, // containers 1..9
    60,                                 // 10
    120,                                // 11
    120                                 // 12
};
#endif

// Program variables
uint8_t tank = 1;
unsigned long startTimeTank = 0;  // for time in tank
unsigned long motorStartTime = 0; // for mechanical timeout
unsigned long pressedAt = 0;      // for press button
bool finished = false;            // Cycle finished
bool inspection = false;          // Inspection activated
bool firstCycle11 = false;        // First cycle completed on 11th tank
bool firstCycle12 = false;        // First cycle completed on 12th tank
bool holdHandled = false;         // If we processed the button

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
}
void vibOff()
{
  digitalWrite(VIB_PIN, LOW);
}
void moveOn()
{
  digitalWrite(MOVE_PIN, HIGH);
}
void moveOff()
{
  digitalWrite(MOVE_PIN, LOW);
}

// Display helper
void lcdShowStatus(const char *line1, const char *line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);

  // Debugging
  DBGLN(line1);
  DBGLN(line2);
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

  char buffer[17];
  // %d is a placeholder for integers. %02d means "print 2 digits, pad with 0"
  snprintf(buffer, sizeof(buffer), "%d:%02d", hours, displayMins);
  lcdShowStatus("Remaining Time:", buffer);
}

/**
 * Returns true only after the button has been held for 'duration'
 */
bool buttonHeld(uint8_t button, uint32_t duration)
{
  uint8_t state = sensorActive(button); // Assumes LOW when pressed (INPUT_PULLUP)

  if (state)
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
  S_STARTING,
  S_LOWERING,
  S_DOWN,
  S_CHECKING,
  S_RAISING,
  S_UP,
  S_TRANSITIONING,
  S_ERROR
};

// Forward declarations for predicate/process/event functions
bool idlePredicate(id_t d);
void idleProcess(id_t id);
void idleActionChanged(EventArgs e);

bool startingPredicate(id_t id);
void startingActionChanged(EventArgs e);

void loweringProcess(id_t id);
bool loweringPredicate(id_t id);
void loweringActionChanged(EventArgs e);

void downProcess(id_t id);
bool downPredicate(id_t id);
void downActionChanged(EventArgs e);

void upProcess(id_t id);
bool upPredicate(id_t id);
void upActionChanged(EventArgs e);

void raisingProcess(id_t id);
bool raisingPredicate(id_t id);
void raisingActionChanged(EventArgs e);

void checkingProcess(id_t id);
bool checkingPredicate(id_t id);
void checkingActionChanged(EventArgs e);

void transitiningProcess(id_t id);
bool transitiningPredicate(id_t id);
void transitiningActionChanged(EventArgs e);

// Transition table - keep it as readable blocks. Use predicate timers where necessary.
Transition transitions[] = {
    // S_IDLE: wait start button. When pressed -> CHECK_START
    {idlePredicate, S_IDLE, S_STARTING, idleProcess, idleActionChanged},

    // S_STARTING: read tank and prepare to start.
    {startingPredicate, S_IDLE, S_LOWERING, nullptr, startingActionChanged},

    // S_LOWERING: run movement motor until bottom sensor active -> if bottom sensor active -> VIBRATE else ERROR
    {loweringPredicate, S_LOWERING, S_DOWN, loweringProcess, loweringActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    /*S_DOWN: Vibrating for 1 hour
                              Conatiner 10 -> Start first heater.
                              Container 11 -> Start second heater.
                              Container 12 -> If finished then stop vibrating.
                    */
    {downPredicate, S_RAISING, S_CHECKING, downProcess, downActionChanged, CONTAINER_TIME_MS, TRUE_TIMER},

    /*S_CHECKING: Container 1..10 -> continue to raise state
                              Conatiner 11 + 12 -> Two hours instead of 1 hour, so renter the down state.
                              Conatiner 10 -> Renter if first wax sensor is not ready.
                              Conatiner 11 -> Renter if first wax sensor is not ready.
                    */
    {checkingPredicate, S_DOWN, S_RAISING, checkingProcess, checkingActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    // S_RAISING: run movement up until top sensor active OR timeout -> TOP or ERROR
    {raisingPredicate, S_RAISING, S_UP, raisingProcess, raisingActionChanged},

    // S_UP: wait if insepction is acitvitated, otherwise proceed to transition state
    {upPredicate, S_UP, S_TRANSITIONING, upProcess, upActionChanged},

    // S_TRANSITIONING: small delay between tanks, then either go to next tank's logic or to START if finished
    {transitiningPredicate, S_TRANSITIONING, S_STARTING, transitiningProcess, transitiningActionChanged},

    // ERROR (top or down sensors, heat sensors, motor, heaters)
    {nullptr, S_ERROR, S_ERROR, nullptr, nullptr}};

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
void idleActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Enter idle");
    finished = false;
    firstCycle11 = false;
    firstCycle12 = false;
    startTimeTank = 0;
    break;
  case EXIT:
    DBGLN("Exit idle");
    break;
  }
}

bool startingPredicate(id_t id)
{
  if (!sensorActive(SENSOR_TOP))
  {
    lcdShowStatus("Critical Error", "Top sensor");
    fsm.begin(S_ERROR); // Top sensor is not active -> Error
  }

  if (tank == 11 && !digitalRead(HEATER1_PIN))
  {
    lcdShowStatus("Critical Error", "heater1 or sensor1");
    fsm.begin(S_ERROR);
  }
  if (tank == 12 && (!sensorActive(HEATER2_PIN) || !digitalRead(HEATER1_PIN)))
  {
    lcdShowStatus("Critical Error", "heater1 or sensor1");
    DBGLN("Critical error container 12 with no sensor 2 activated");
    fsm.begin(S_ERROR);
  }
  if (finished)
    return false; // Finished cycle -> false

  return true;
}
void startingActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
  {
    startTimeTank = 0;
    uint8_t currentTank = readTankSelector();
    tank = currentTank;
    char buf1[17];
    char buf2[17];
    snprintf(buf1, sizeof(buf1), "Tank: %u", currentTank);
    snprintf(buf2, sizeof(buf2), "Starting...");
    lcdShowStatus(buf1, buf2);
    DBG("Entering tank: ");
    DBGLN(currentTank);
    break;
  }
  case EXIT:
    break;
  }
}

void loweringProcess(id_t id)
{
  if (!digitalRead(MOVE_PIN))
    moveOn();

  // check mechanical timeout
  if (motorStartTime == 0)
    motorStartTime = millis();
}
bool loweringPredicate(id_t id)
{
  // if bottom sensor active -> true to move to VIBRATE
  if (sensorActive(SENSOR_BOTTOM) && !sensorActive(SENSOR_TOP))
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
    motorStartTime = 0;
    DBGLN("Lower timeout -> ERROR");
    lcdShowStatus("ERROR", "MOTOR OVER TIME");
    fsm.begin(S_ERROR);
    return false; // remain or exit to error as FSM sets state elsewhere
  }

  return false; // continue lowering
}
void loweringActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Lowering..");
    lcdShowStatus("Lowering", "");
    break;

  case EXIT:
    DBGLN("Exit Lowering");
    break;
  }
}

bool downPredicate(id_t id)
{
  if (buttonHeld(START_BUTTON, START_BUTTON_DELAY_MS))
  {
    DBGLN("Raising to top");
    lcdShowStatus("Button Pressed", "Raising...");
    inspection = true;
    startTimeTank = 0;
    return false;
  }
  return true;
}
void downProcess(id_t id)
{
  if (tank == 12 && finished)
  {
    vibOff();
    lcdShowStatus("Finished", "Press Run...");
    DBGLN("Finished");
    DBGLN("Press Run to continue...");
    return;
  }

  if (!digitalRead(VIB_PIN))
  {
    vibOn();
    DBGLN("Start Vibrating");
  }

  if (tank == 10 && !sensorActive(SENSOR_WAX1) && !digitalRead(HEATER1_PIN))
  {
    digitalWrite(HEATER1_PIN, HIGH);
    DBGLN("Start First Heater");
  }
  else if (tank == 11 && !sensorActive(SENSOR_WAX2) && !digitalRead(HEATER2_PIN))
  {
    digitalWrite(HEATER2_PIN, HIGH);
    DBGLN("Start Second Heater");
  }

  // Print remaining time on lcd screen.
  printRemainingTimeForTank(tank);

  return;
}
void downActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    if (startTimeTank == 0)
      startTimeTank = millis();
    DBGLN("Enter down state");
    break;

  case EXIT:
    DBGLN("Exiting down state");
    break;
  }
}

void checkingProcess(id_t id)
{
  if (tank == 11)
  {
    firstCycle11 = true;
    return;
  }
  if (tank == 12)
  {
    if (firstCycle12)
    {
      finished = true;
      DBGLN("Cycle finsihed");
      return;
    }
    firstCycle12 = true;
  }
}
bool checkingPredicate(id_t id)
{
  if (tank == 10 && !sensorActive(SENSOR_WAX1))
  {
    DBGLN("Returning to down state because wax sensor is not ");
    lcdShowStatus("Waiting for", "Wax 1 Heat...");
    return false;
  }
  else if (tank == 11 && (!sensorActive(SENSOR_WAX2) || !firstCycle11))
  {
    DBGLN("Moving to second hour");
    return false;
  }
  else if (tank == 12)
  {
    DBGLN("Returning to down state 12 containers");
    return false;
  }
  startTimeTank = 0;
  return true;
}
void checkingActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Enter processing state");
    if (digitalRead(VIB_PIN))
      vibOff();
    break;

  case EXIT:
    DBGLN("Exit processing state");
    break;
  }
}

void upProcess(id_t id)
{
  if (inspection && !digitalRead(MOVE_PIN))
  {
    moveOff();
    motorStartTime = 0;
    DBGLN("Top Position");
    DBGLN("Press Run to continue");
    lcdShowStatus("Top inspection", "Press Run");
    return;
  }
  return;
}
bool upPredicate(id_t id)
{
  if (!inspection)
    return true;
  if (buttonHeld(START_BUTTON, START_BUTTON_DELAY_MS))
  {
    inspection = false;
    return false;
  }
  return false;
}
void upActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("UP state");
    break;

  case EXIT:
    DBGLN("Exiting UP state");
    break;
  }
}

void raisingProcess(id_t id)
{
  if (!digitalRead(MOVE_PIN))
    moveOn();

  // check mechanical timeout
  if (motorStartTime == 0)
    motorStartTime = millis();
}
bool raisingPredicate(id_t id)
{

  // if top sensor active -> true to move to VIBRATE
  if (sensorActive(SENSOR_TOP) && !sensorActive(SENSOR_BOTTOM))
  {
    moveOff();
    motorStartTime = 0;
    DBGLN("Reached Top");
    return true;
  }
  // mechanical timeout
  if (motorStartTime && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
  {
    // timeout -> error
    moveOff();
    DBGLN("Raise timeout -> ERROR");
    lcdShowStatus("ERROR", "MOTOR OVER TIME");
    fsm.begin(S_ERROR);
    return false; // remain or exit to error as FSM sets state elsewhere
  }

  return false; // continue Raising
}
void raisingActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Raising..");
    lcdShowStatus("Raising", "");
    break;

  case EXIT:
    DBGLN("Exit Raising state");
    lcdShowStatus("Reached top", "");
    break;
  }
}

void transitiningProcess(id_t id)
{
  if (!digitalRead(MOVE_PIN))
    moveOn();

  // check mechanical timeout
  if (motorStartTime == 0)
    motorStartTime = millis();
}
bool transitiningPredicate(id_t id)
{
  if (!sensorActive(SENSOR_TOP))
  {
    lcdShowStatus("Critical Error", "Top sensor");
    fsm.begin(S_ERROR); // Top sensor is not active -> Error
  }
  if (motorStartTime && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
  {
    // timeout -> error
    moveOff();
    motorStartTime = 0;
    DBGLN("Lower timeout -> ERROR");
    fsm.begin(S_ERROR);
    return false; // remain or exit to error as FSM sets state elsewhere
  }

  uint8_t currentContainer = readTankSelector();
  if (currentContainer != tank)
  {
    tank = currentContainer;
    return true;
  }
  return false;
}
void transitiningActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Entering Transition State");
    break;

  case EXIT:
    DBGLN("Exiting Transition State");
    break;
  }
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
  pinMode(SENSOR_WAX2, INPUT_PULLUP);
  pinMode(SENSOR_WAX1, INPUT_PULLUP);
  pinMode(SENSOR_BOTTOM, INPUT_PULLUP);
  pinMode(SENSOR_TOP, INPUT_PULLUP);

  pinMode(START_BUTTON, INPUT_PULLUP); // use internal pullup and interpret LOW as pressed

  for (uint8_t i = 0; i < 4; ++i)
    pinMode(PIN_ID_BITS[i], INPUT_PULLUP);
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    DBGLN("Serial debug enabled");
#endif
  setupPins();
  // TODO: Handle sensors error
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // start FSM in IDLE
  fsm.begin(S_IDLE);
}

void loop()
{ // TODO: Handle LCD Flicker
  // Execute FSM regularly
  fsm.execute();
  delay(10);
}
