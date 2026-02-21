// #define DEBUG // Uncomment to enable Serial output for debugging
// #define TEST  // Uncomment to enable fast timers for testing

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
      D12 -> movement motor (MOVE_PIN)
      D11 -> vibration motor (VIB_PIN)
      D10 -> heater 2 (HEATER2_PIN)
      D9  -> heater 1 (HEATER1_PIN)

    Sensor inputs (active LOW): D5..D8
      D8 -> top-level sensor (TOP_SENSOR_PIN)
      D7 -> bottom-level sensor (BOTTOM_SENSOR_PIN)
      D6 -> wax-ready sensor 2 (WAX2_SENSOR_PIN)
      D5 -> wax-ready sensor 1 (WAX1_SENSOR_PIN)

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
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal_I2C.h> // 1.1.2

// ========================= PIN MAP (as provided) =========================
// Power outputs
const uint8_t MOVE_PIN = 12;    // D12 - Processing vibration motor
const uint8_t VIB_PIN = 11;     // D11 - Relay enable for AC motor
const uint8_t HEATER2_PIN = 10; // D10   - Heater 1
const uint8_t HEATER1_PIN = 9;  // D9  - Heater 2 (power unit)
// Sesnors &  thermostats (Active LOW: 0 = active)
const uint8_t SENSOR_TOP = 8;    // D8
const uint8_t SENSOR_BOTTOM = 7; // D7
const uint8_t SENSOR_WAX2 = 6;   // D6
const uint8_t SENSOR_WAX1 = 5;   // D5

// Container ID (A0..A3)
const uint8_t PIN_ID_BITS[4] = {A0, A1, A2, A3}; // bit0 LSB .. bit3 MSB

// UI
const uint8_t START_BUTTON = 2; // D2

// LCD (I2C)
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ========================= CONFIGURATION & TIME CONSTANTS =========================
#ifdef TEST
const unsigned long ONE_MIN_MS = 1000UL;               // 1 second = 1 "minute" for testing
const unsigned long MIN_DWELL_MIN = 10UL;              // allow 10 seconds in test
const unsigned long TANK_TIME_MS = 10UL * 1000UL;      // stays down for 10 seconds while vibrating
const unsigned long TANK_STABILITY_THRESHOLD = 2000UL; // 2 seconds
const unsigned long MOVE_TIMEOUT_MS = 10UL * 1000UL;   // 10 seconds - motion safety timeout
#else
const unsigned long ONE_MIN_MS = 60UL * 1000UL;          // 1 real minute
const unsigned long MIN_DWELL_MIN = 60UL;                // production min dwell in minutes
const unsigned long TANK_TIME_MS = 60UL * 60UL * 1000UL; // Normaly stays down for 1 hour while vibrating
const unsigned long TANK_STABILITY_THRESHOLD = 200UL;    // ms
const unsigned long MOVE_TIMEOUT_MS = 30UL * 1000UL;     // 30 seconds - motion safety timeout

#endif

const unsigned long MOTOR_SWITCH_DELAY_MS = 1000UL;       // 1 second - motor swithc saf
const unsigned long START_BUTTON_DELAY_MS = 2UL * 1000UL; // Idle state - 2 seconds
const unsigned long DEBOUNCE_DELAY_MS = 20;               // debounce time for sensors = 20 ms
const uint8_t TANK_12 = 12;

// Program variables
unsigned long startTimeTank = 0;  // for time in tank
unsigned long motorStartTime = 0; // for mechanical timeout
unsigned long pressedAt = 0;      // for press button
uint8_t lastStableTank = 1;
uint8_t pendingTank = 1;
uint8_t currentCycle = 1;
unsigned long tankStabilityTime = 0;
unsigned long lastPrintTimeTank = 0; // Last time printed remaining time for tank...
bool finished = false;               // Cycle finished
bool inspection = false;             // Inspection activated
bool holdHandled = false;            // If we processed the button
bool tankChanged = false;
bool isMoving = false;    // true = ON
bool isVibrating = false; // true = ON
bool isHeating1 = false;  // true = ON
bool isHeating2 = false;  // true = ON
bool tankException = false;
bool waitingWaxMelt = false;

// Utility: read tank number from A0..A3 as digital inputs (0..15) then +1 (1..16) clamp to 1..12
void readTankID()
{
  uint8_t currentRead = 0;
  for (uint8_t i = 0; i < 4; ++i)
  {
    uint8_t v = (digitalRead(PIN_ID_BITS[i]) == HIGH) ? 1 : 0;
    currentRead |= (v << i);
  }
  if (currentRead < 1 || currentRead > 12)
  {
    tankException = true;
    DBGLN("Wrong Tank ID");
    DBGLN(currentRead);
    return;
  }
  if (currentRead != pendingTank)
  {
    pendingTank = currentRead;
    tankStabilityTime = millis();
    tankChanged = false;
    return;
  }
  if (lastStableTank != pendingTank && (millis() - tankStabilityTime) >= TANK_STABILITY_THRESHOLD)
  {
    lastStableTank = pendingTank;
    tankStabilityTime = 0;
    tankChanged = true;
    return;
  }
}

// Sensor helpers (active LOW). Return true when sensor is active (LOW) or when a fail-safe (treated as active).
bool sensorActive(uint8_t pin)
{
  return digitalRead(pin) == LOW;
}

// Define a structure to track sensor state
struct DebouncedSensor
{
  uint8_t pin;
  unsigned long lastLowTime;
  bool stableActive; // Added to store the "last known" state

  void update()
  {
    bool rawReading = (digitalRead(pin) == LOW);

    if (rawReading)
    {
      if (lastLowTime == 0)
        lastLowTime = millis();
      if (millis() - lastLowTime >= DEBOUNCE_DELAY_MS)
      {
        stableActive = true;
      }
    }
    else
    {
      lastLowTime = 0;
      stableActive = false;
    }
  }

  bool isActive()
  {
    return stableActive;
  }
};

// Initialize your specific sensors
DebouncedSensor topLimit = {SENSOR_TOP, 0};
DebouncedSensor bottomLimit = {SENSOR_BOTTOM, 0};
DebouncedSensor wax1Ready = {SENSOR_WAX1, 0};
DebouncedSensor wax2Ready = {SENSOR_WAX2, 0};

struct TankProfile
{
  unsigned int dwellMinutes;
  uint8_t requiredHeater; // 0: None, 1: Heater1, 2: Heater2, 3: Both
  uint8_t requiredWax;    // 0: None, 1: Wax1, 2: Wax2, 3: Both
  uint8_t cycles;         // 1: Normal, 2: Double dwell (for tanks 11, 12)
} __attribute__((packed));

// Now define your 12 tanks in one clean table
const TankProfile tanks[13] PROGMEM = {
    {0, 0, 0, 1},  // Tank 0 (unused)
    {60, 0, 0, 1}, // Tanks 1-9: No heat, no wax sensor, 1 cycle
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 0, 0, 1},
    {60, 1, 0, 1},  // Tank 10: Heater 1, Wax Sensor 1, 1 cycle
    {120, 3, 1, 2}, // Tank 11: Both Heaters, Wax Sensor 1, 2 cycles
    {120, 3, 3, 2}  // Tank 12: Both Heaters, Both Sensors, 2 cycles
};

uint8_t safeNumber(uint8_t number)
{
  if (number > 12 || number < 1)
    return 1;

  return number;
}
// Accessors for PROGMEM table
static inline uint16_t getDwellMinutes(uint8_t idx)
{
  return (uint16_t)pgm_read_word(&(tanks[idx].dwellMinutes));
}
static inline uint8_t getRequiredHeater(uint8_t idx)
{
  return (uint8_t)pgm_read_byte(&(tanks[idx].requiredHeater));
}
static inline uint8_t getRequiredWaxSensor(uint8_t idx)
{
  return (uint8_t)pgm_read_byte(&(tanks[idx].requiredWax));
}
static inline uint8_t getNextRequiredSensor(uint8_t idx)
{
  return (uint8_t)pgm_read_byte(&(tanks[safeNumber(++idx)].requiredWax));
}
static inline uint8_t getCycles(uint8_t idx)
{
  return (uint8_t)pgm_read_byte(&(tanks[idx].cycles));
}

// Motor control helpers
void moveOn()
{
  if (isMoving)
    return;

  digitalWrite(MOVE_PIN, HIGH);
  isMoving = true;
  DBGLN("Moving on");
}
void moveOff()
{
  if (!isMoving)
    return;

  digitalWrite(MOVE_PIN, LOW);
  isMoving = false;
  DBGLN("Moving off");
}
void vibOn()
{
  if (isVibrating)
    return;

  digitalWrite(VIB_PIN, HIGH);
  isVibrating = true;
  DBGLN("Vibrating on");
}
void vibOff()
{
  if (!isVibrating)
    return;

  digitalWrite(VIB_PIN, LOW);
  isVibrating = false;
  DBGLN("Vibrating off");
}

void heaterOn1()
{
  if (isHeating1)
    return;

  digitalWrite(HEATER1_PIN, HIGH);
  isHeating1 = true;
  DBGLN("Start First Heater");
}
void heaterOn2()
{
  if (isHeating2)
    return;

  digitalWrite(HEATER2_PIN, HIGH);
  isHeating2 = true;
  DBGLN("Start Second Heater");
}
void heaterOff1()
{
  if (isHeating1)
    return;

  digitalWrite(HEATER1_PIN, LOW);
  isHeating1 = false;
  DBGLN("Stop First Heater");
}
void heaterOff2()
{
  if (!isHeating2)
    return;

  digitalWrite(HEATER2_PIN, LOW);
  isHeating2 = false;
  DBGLN("Stop Second Heater");
}

void manageHeaters(uint8_t tank)
{
  uint8_t h = getRequiredHeater(tank);
  (h & 1) ? heaterOn1() : heaterOff1();
  (h & 2) ? heaterOn2() : heaterOff2();
}

void outputsKill()
{
  moveOff();
  vibOff();
  heaterOff1();
  heaterOff2();
}

// Helper for F() strings
void lcdPrintPadded(const __FlashStringHelper *text)
{
  DBGLN(text);
  lcd.print(text);
  // strlen_P is the version of strlen that reads from Flash
  int len = strlen_P((PGM_P)text);
  for (int i = len; i < 16; i++)
  {
    lcd.print(F(" "));
  }
}

// Helper for RAM strings
void lcdPrintPadded(const char *text)
{
  DBGLN(text);
  lcd.print(text);
  int len = strlen(text);
  for (int i = len; i < 16; i++)
  {
    lcd.print(F(" "));
  }
}
// Version for F() macro strings (Flash memory)
void lcdShowStatus(const __FlashStringHelper *line1, const __FlashStringHelper *line2)
{
  lcd.setCursor(0, 0);
  lcdPrintPadded(line1);
  lcd.setCursor(0, 1);
  lcdPrintPadded(line2);
}

// Version for RAM-based strings (like buffers or variables)
void lcdShowStatus(const char *line1, const char *line2)
{
  lcd.setCursor(0, 0);
  lcdPrintPadded(line1);
  lcd.setCursor(0, 1);
  lcdPrintPadded(line2);
}

void LcdShowTank(uint8_t tank)
{
  lcd.print(F("Tank: ")); // Print the label from Flash
  lcd.print(tank);        // Print the variable directly

  // Manual Padding: Clear the rest of the line (16 chars total)
  // "Tank: " is 6 chars, lastStableTank is 1 or 2 chars.
  int usedChars = (tank < 10) ? 7 : 8;
  for (int i = usedChars; i < 16; i++)
  {
    lcd.print(F(" "));
  }
  DBG("Tank: ");
  DBGLN(tank);
}

void lcdShowStatusTank(const __FlashStringHelper *text)
{
  lcd.setCursor(0, 0);
  LcdShowTank(lastStableTank);

  lcd.setCursor(0, 1);
  lcdPrintPadded(text);
}

// Print remaining time for current tank
void printRemainingTimeForTank(uint8_t tank)
{
  // 1. Throttle the update rate (once per minute or second)
  if (lastPrintTimeTank != 0 && millis() - lastPrintTimeTank < ONE_MIN_MS)
    return;

  lastPrintTimeTank = millis();

  // 2. Get total duration from our Data Table
  // If the tank is 0 or out of bounds, use a safety fallback
  unsigned int totalMins = (tank >= 1 && tank <= 12) ? getDwellMinutes(tank) : MIN_DWELL_MIN;

  // 3. Calculate Elapsed and Remaining
  unsigned long totalMs = (unsigned long)totalMins * ONE_MIN_MS;
  unsigned long elapsedMs = millis() - startTimeTank;

  // Use long to prevent underflow wrap-around errors
  long remainingMs = (long)totalMs - (long)elapsedMs;
  if (remainingMs < 0)
    remainingMs = 0;

  // 4. Conversion
  unsigned long remainingTotalMins = remainingMs / ONE_MIN_MS;
  unsigned int hours = remainingTotalMins / 60;
  unsigned int displayMins = remainingTotalMins % 60;

  // 5. LCD Update
  lcd.setCursor(0, 0);
  LcdShowTank(tank);

  lcd.setCursor(0, 1);
  lcd.print(F("Time "));
  lcd.print(hours);
  lcd.print(F(":"));
  if (displayMins < 10)
    lcd.print(F("0"));
  lcd.print(displayMins);

  // Padding to clear old characters
  for (int i = (hours > 9 ? 9 : 8); i < 16; i++)
  {
    lcd.print(F(" "));
  }
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
      pressedAt = 0;
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
  S_PRE_DOWN,
  S_DOWN,
  S_CHECKING,
  S_PRE_RAISING,
  S_RAISING,
  S_UP,
  S_TRANSITIONING,
  S_ERROR
};

// Forward declarations for predicate/process/event functions
bool idlePredicate(id_t id);
void idleActionChanged(EventArgs e);

bool startingPredicate(id_t id);
void startingActionChanged(EventArgs e);

bool loweringPredicate(id_t id);
void loweringActionChanged(EventArgs e);

void downProcess(id_t id);
bool downPredicate(id_t id);
void downActionChanged(EventArgs e);

void upProcess(id_t id);
bool upPredicate(id_t id);
void upActionChanged(EventArgs e);

bool raisingPredicate(id_t id);
void raisingActionChanged(EventArgs e);

bool checkingPredicate(id_t id);
void checkingActionChanged(EventArgs e);

bool transitiningPredicate(id_t id);
void transitiningActionChanged(EventArgs e);

void errorActionChanged(EventArgs e);

void onActionChanged(EventArgs e);
// Transition table - keep it as readable blocks. Use predicate timers where necessary.
Transition transitions[] = {
    // S_IDLE: wait start button. When pressed -> CHECK_START
    {idlePredicate, S_IDLE, S_STARTING, nullptr, idleActionChanged},

    // S_STARTING: read tank and prepare to start.
    {startingPredicate, S_IDLE, S_LOWERING, nullptr, startingActionChanged},

    // S_LOWERING: run movement motor until bottom sensor active -> if bottom sensor active -> DOWN else ERROR
    {loweringPredicate, S_LOWERING, S_PRE_DOWN, nullptr, loweringActionChanged},

    // S_PRE_DOWN: Just wait for MOTOR_SWITCH_DELAY_MS before moving to next state
    // (safe switching between moving and vibrating)
    {nullptr, S_PRE_DOWN, S_DOWN, nullptr, onActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    /*S_DOWN: Vibrating for 1 hour
            Conatiner 10 -> Start first heater.
            Container 11 -> Start second heater.
            Container 12 -> If finished then stop vibrating.
            */
    {downPredicate, S_PRE_RAISING, S_CHECKING, downProcess, downActionChanged, TANK_TIME_MS, TRUE_TIMER},

    /*S_CHECKING: Container 1..10 -> continue to raise state
            Conatiner 11 + 12 -> Two hours instead of 1 hour, so renter the down state.
            Conatiner 10 -> Renter if first wax sensor is not ready.
            Conatiner 11 -> Renter if first wax sensor is not ready.
            */
    {checkingPredicate, S_DOWN, S_PRE_RAISING, nullptr, checkingActionChanged},

    // S_PRE_RAISING: Just wait for MOTOR_SWITCH_DELAY_MS before moving to next state
    // (safe switching between moving and vibrating)
    {nullptr, S_PRE_RAISING, S_RAISING, nullptr, onActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    // S_RAISING: run movement up until top sensor active OR timeout -> TOP or ERROR
    {raisingPredicate, S_RAISING, S_UP, nullptr, raisingActionChanged},

    // S_UP: wait if insepction is acitvitated, otherwise proceed to transition state
    {upPredicate, S_UP, S_TRANSITIONING, upProcess, upActionChanged},

    // S_TRANSITIONING: small delay between tanks, then either go to next tank's logic or to START if finished
    {transitiningPredicate, S_TRANSITIONING, S_STARTING, nullptr, transitiningActionChanged},

    // ERROR (top or down sensors, heat sensors, motor, heaters)
    {nullptr, S_ERROR, S_ERROR, nullptr, errorActionChanged}};

const uint8_t numberOfTransitions = sizeof(transitions) / sizeof(Transition);

FiniteState fsm(transitions, numberOfTransitions);

// Implementation: Predicates, Processes and Events
bool idlePredicate(id_t id)
{
  if (buttonHeld(START_BUTTON, START_BUTTON_DELAY_MS))
  {
    DBGLN("Start button pressed");
    return true;
  }
  return false;
}

void idleActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    outputsKill();

    readTankID();
    DBGLN("Enter idle");
    lcdShowStatus(F("Status: Idle"), F("Press Start"));
    finished = false;
    break;
  case EXIT:
    DBGLN("Exit idle");
    break;
  }
}

bool startingPredicate(id_t id)
{
  if (!topLimit.isActive())
  {
    lcdShowStatus(F("Critical Error"), F("Top sensor"));
    fsm.begin(S_ERROR); // Top sensor is not active -> Error
  }
  bool waxReady = true;
  uint8_t s = getRequiredWaxSensor(lastStableTank);
  if ((s & 1) && !wax1Ready.isActive())
  {
    lcdShowStatus(F("Critical Error"), F("H1 or S1"));
    fsm.begin(S_ERROR);
  }
  if ((s & 2) && !wax2Ready.isActive())
  {
    lcdShowStatus(F("Critical Error"), F("H1 H2 or S1 S2"));
    DBGLN("Critical error container 12 without melted wax");
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
    readTankID();

    lcdShowStatusTank(F("Starting...")); // Uses F() to keep text in Flash

    DBG("Entering tank: ");
    DBGLN(lastStableTank);
    break;
  }
  case EXIT:
    tankChanged = false;
    break;
  }
}

bool loweringPredicate(id_t id)
{
  // if bottom sensor active -> true, so move to DOWN
  if (bottomLimit.isActive() && !topLimit.isActive())
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
    lcdShowStatus(F("ERROR"), F("MOTOR OVER TIME"));
    fsm.begin(S_ERROR);
  }

  return false; // continue lowering
}
void loweringActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    if (!isMoving)
    {
      moveOn();
      motorStartTime = millis();
    }
    lcdShowStatusTank(F("Lowering..")); // Uses F() to keep text in Flash
    break;

  case EXIT:
    DBGLN("Exit Lowering");
    break;
  }
}

bool downPredicate(id_t id)
{
  if (!bottomLimit.isActive() || topLimit.isActive())
  {
    lcdShowStatus(F("ERROR"), F("TOP or LOW sensor"));
    fsm.begin(S_ERROR);
  }

  if (buttonHeld(START_BUTTON, START_BUTTON_DELAY_MS))
  {
    vibOff();
    inspection = true;
    DBGLN("Raising to top");
    lcdShowStatus(F("Button Pressed"), F("Raising..."));
    return false;
  }
  if (waitingWaxMelt)
  {
    uint8_t s = getNextRequiredSensor(lastStableTank);
    if ((s & 2) && wax2Ready.isActive())
    {
      waitingWaxMelt = false;
      return false;
    }
    if ((s & 1) && wax1Ready.isActive())
    {
      waitingWaxMelt = false;
      return false;
    }
  }
  return true;
}
void downProcess(id_t id)
{
  if (lastStableTank == TANK_12 && finished)
  {
    if (isVibrating)
    {
      vibOff();
      lcdShowStatus(F("Finished"), F("Press Run..."));
      DBGLN("Finished");
      DBGLN("Press Run to continue...");
    }
    return;
  }

  vibOn();

  if (waitingWaxMelt)
    return;

  // Print remaining time on lcd screen.
  printRemainingTimeForTank(lastStableTank);

  return;
}
void downActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    if (startTimeTank == 0)
      startTimeTank = millis();

    if (waitingWaxMelt)
    {
      lcdShowStatusTank(F("Waiting Wax"));
    }

    manageHeaters(lastStableTank);

    DBGLN("Enter down state");
    break;

  case EXIT:
    lastPrintTimeTank = 0;
    DBGLN("Exiting down state");
    break;
  }
}

bool checkingPredicate(id_t id)
{
  // 1. Check Multi-cycle Logic (2-hour tanks)
  if (currentCycle < getCycles(lastStableTank))
  {
    currentCycle++;
    return false; // Stay in DOWN state
  }

  // 2. Check Wax Readiness
  bool waxReady = true;
  uint8_t s = getNextRequiredSensor(lastStableTank);
  if ((s & 1) && !wax1Ready.isActive())
    waxReady = false;
  if ((s & 2) && !wax2Ready.isActive())
    waxReady = false;

  if (!waxReady)
  {
    waitingWaxMelt = true;
    return false; // Stay in DOWN state
  }

  waitingWaxMelt = false;

  // 3. Final Tank Check
  if (lastStableTank == TANK_12)
  {
    finished = true;
    vibOff();
    return false;
  }

  currentCycle = 1; // Reset for next tank
  startTimeTank = 0;
  return true; // Proceed to RAISE
}
void checkingActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Enter checking state");
    break;

  case EXIT:
    DBGLN("Exit checking state");
    break;
  }
}

void upProcess(id_t id)
{
  if (inspection && isMoving)
  {
    moveOff();
    motorStartTime = 0;
    DBGLN("Top Position");
    DBGLN("Press Run to continue");
    lcdShowStatus(F("Top inspection"), F("Press Run"));
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
    DBGLN("Top state");
    break;

  case EXIT:
    DBGLN("Exiting Top state");
    break;
  }
}

bool raisingPredicate(id_t id)
{
  // if top sensor active -> true to move to TRANSITION
  if (topLimit.isActive() && !bottomLimit.isActive())
  {
    DBGLN("Reached Top");
    return true;
  }
  // mechanical timeout
  if (motorStartTime && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
  {
    // timeout -> error
    moveOff();
    motorStartTime = 0;
    DBGLN("Raise timeout -> ERROR");
    lcdShowStatus(F("ERROR"), F("MOTOR OVER TIME"));
    fsm.begin(S_ERROR); // remain or exit to error as FSM sets state elsewhere
  }

  return false; // continue Raising
}
void raisingActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    DBGLN("Raising..");
    // check mechanical timeout
    if (!isMoving)
    {
      moveOn();
      motorStartTime = millis();
    }

    lcdShowStatusTank(F("Raising"));

    break;

  case EXIT:

    DBGLN("Exit Raising state");

    lcdShowStatusTank(F("Reached top"));

    break;
  }
}

bool transitiningPredicate(id_t id)
{
  if (!topLimit.isActive())
  {
    lcdShowStatus(F("Critical Error"), F("Top sensor"));
    fsm.begin(S_ERROR); // Top sensor is not active -> Error
  }
  if (isMoving && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
  {
    // timeout -> error
    moveOff();
    motorStartTime = 0;
    DBGLN("Transition timeout -> ERROR");
    lcdShowStatus(F("Critical Error"), F("Transition timeout"));
    fsm.begin(S_ERROR); // remain or exit to error as FSM sets state elsewhere
  }

  readTankID();

  return tankChanged;
}
void transitiningActionChanged(EventArgs e)
{
  switch (e.action)
  {
  case ENTRY:
    // check mechanical timeout
    if (!isMoving)
    {
      moveOn();
      motorStartTime = millis();
    }

    DBGLN("Entering Transition State");
    lcdShowStatus(F("Transition State"), F(""));
    break;

  case EXIT:
    DBGLN("Exiting Transition State");
    tankChanged = false;
    break;
  }
}

void errorActionChanged(EventArgs e)
{
  if (e.action == ENTRY)
  {
    // KILL EVERYTHING
    outputsKill();

    DBGLN("!!! SAFETY SHUTDOWN !!!");
  }
}

void onActionChanged(EventArgs e)
{
  if (e.action == EXIT)
  {
    startTimeTank = 0;
    vibOff();
  }
  return;
}
// Setup and loop
void handleSensorsFailure()
{ // TODO: Handle sensors' errors..
  bool topSensor = topLimit.isActive();
  bool bottomSensor = bottomLimit.isActive();
  bool heatSensor_1 = wax1Ready.isActive();
  bool heatSensor_2 = wax2Ready.isActive();
  DBGLN(topSensor);
  DBGLN(bottomSensor);
  DBGLN(heatSensor_1);
  DBGLN(heatSensor_2);
}
// ========================= TASKS & SCHEDULER =========================
// Tasks run from a cooperative scheduler tick
unsigned long lastTick = 0;
const unsigned long TICK_MS = 10UL; // 10ms tick
bool lastLoopHealthy = false;

void sensorTask()
{
  topLimit.update();
  bottomLimit.update();
  wax1Ready.update();
  wax2Ready.update();
}

void safetyTask()
{
  if (fsm.id == S_ERROR)
    return;

  if (topLimit.isActive() && bottomLimit.isActive())
  {
    lcdShowStatus(F("ERROR"), F("TOP or LOW sensors"));
    fsm.begin(S_ERROR);
  }

  if (tankException)
  {
    lcdShowStatus(F("ERROR"), F("Tank read"));
    fsm.begin(S_ERROR);
  }
}

void fsmTask() { fsm.execute(); }

void healthTask()
{ // run last - decide if we reset WDT
  // simple heuristic: not in error state and at least one sensor updated recently
  lastLoopHealthy = (fsm.id != S_ERROR);

  if (lastLoopHealthy)
    wdt_reset();
}

// ========================= SETUP & LOOP =========================
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

  // Sensor inputs - active LOW
  pinMode(SENSOR_WAX2, INPUT_PULLUP);
  pinMode(SENSOR_WAX1, INPUT_PULLUP);
  pinMode(SENSOR_BOTTOM, INPUT_PULLUP);
  pinMode(SENSOR_TOP, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  for (uint8_t i = 0; i < 4; i++)
    pinMode(PIN_ID_BITS[i], INPUT);
}

void setup()
{
  // #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    ;
  // #endif
  setupPins();
  Wire.begin();
  lcd.init();
  lcd.backlight();
  readTankID();
  fsm.begin(S_IDLE);
  wdt_enable(WDTO_2S);
}

void loop()
{
  unsigned long now = millis();
  if (now - lastTick >= TICK_MS)
  {

    unsigned long start = micros(); // 🔹 start timing
    lastTick = now;
    sensorTask();
    safetyTask();
    fsmTask();
    healthTask();

    unsigned long duration = micros() - start; // 🔹 end timing
    if (duration > 1000)
      Serial.println(duration); // 🔹 print µs
  }
}
