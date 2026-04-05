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
    Power outputs: D5..D8
      D8 -> movement motor (MOVE_PIN)
      D7 -> vibration motor (VIB_PIN)
      D6 -> heater 2 (HEATER2_PIN)
      D5  -> heater 1 (HEATER1_PIN)

    Sensor inputs (active LOW): A0, A1, D3, D4
      A0 -> top-level sensor (TOP_SENSOR_PIN)
      A1 -> bottom-level sensor (BOTTOM_SENSOR_PIN)
      D4 -> wax-ready sensor 2 (WAX2_SENSOR_PIN)
      D3 -> wax-ready sensor 1 (WAX1_SENSOR_PIN)

    Tank selector: D9..D12 (4-bit binary) Active = HIGH

    Start/Stop button: D2
    Skip to next tank button: D1
    Raise/Continue button: D0


    I2C LCD: A4 (SDA), A5 (SCL)

  Notes / safety:
    - Sensors are treated active when digitalRead == LOW.
    - Mechanical movement timeout: 30 seconds (if motor runs without reaching
  target sensor).
    - 1 second delay when switching between vibration and movement motors.
    - Fail-safe: Any sensor reading LOW unexpectedly stops motion and sets ERROR
  state.

  Libraries required:
    - Finite-State 1.6.0 (https://github.com/MicroBeaut/Finite-State) added
  locally...
    - LiquidCrystal_I2C (for I2C LCD, typical address 0x27)
*/

#include <FiniteState.h>       // 1.6.0
#include <LiquidCrystal_I2C.h> // 1.1.2
#include <Wire.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
// ========================= PIN MAP (as provided) =========================
// Power outputs
const uint8_t MOVE_PIN = 8;    // D8 - Processing vibration motor
const uint8_t VIB_PIN = 7;     // D7 - Relay enable for AC motor
const uint8_t HEATER1_PIN = 6; // D6  - Heater 1
const uint8_t HEATER2_PIN = 5; // D5 - Heater 2
// Sesnors &  thermostats (Active LOW: 0 = active)
const uint8_t SENSOR_TOP = A0;    // A0
const uint8_t SENSOR_BOTTOM = A1; // A1
const uint8_t SENSOR_WAX1 = 3;    // D3
const uint8_t SENSOR_WAX2 = 4;    // D4

// Container ID (B9..B12)
const uint8_t PIN_ID_BITS[4] = {9, 10, 11, 12}; // bit0 LSB .. bit3 MSB

// UI
const uint8_t START_BUTTON = 2; // D2
const uint8_t SKIP_BUTTON = 1;  // D1
const uint8_t RAISE_BUTTON = 0; // D0

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

const unsigned long MOTOR_SWITCH_DELAY_MS = 1000UL; // 1 second - motor swithc saf
const unsigned long VERIFICATION_DELAY_MS = 2UL * 1000UL;
const unsigned long BUTTON_DELAY_MS = 3UL * 1000UL; // Idle state - 2 seconds
const unsigned long DEBOUNCE_DELAY_MS = 20;         // debounce time for sensors = 20 ms
const uint8_t TANK_12 = 12;

// Program variables
unsigned long startTimeTank = 0;  // for time in tank
unsigned long motorStartTime = 0; // for mechanical timeout
uint8_t lastStableTank = 1;
uint8_t pendingTank = 1;
uint8_t currentCycle = 1;
unsigned long tankStabilityTime = 0;
unsigned long lastPrintTimeTank = 0; // Last time printed remaining time for tank...
bool finished = false;               // Cycle finished
bool inspection = false;             // Inspection activated
bool tankChanged = false;
bool isMoving = false;    // true = ON
bool isVibrating = false; // true = ON
bool isHeating1 = false;  // true = ON
bool isHeating2 = false;  // true = ON
bool tankException = false;
bool waitingWaxMelt = false;

// Sensor helpers (active LOW). Return true when sensor is active (LOW) or when
// a fail-safe (treated as active).
bool sensorActive(uint8_t pin) { return digitalRead(pin) == LOW; }

// Utility: read tank number from A0..A3 as digital inputs (0..15) then +1
// (1..16) clamp to 1..12
void syncTankID()
{
    uint8_t currentRead = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        uint8_t v = digitalRead(PIN_ID_BITS[i]);
        currentRead |= (v << i);
    }

    if (currentRead != pendingTank)
    {
        pendingTank = currentRead;
        tankStabilityTime = millis();
        tankChanged = false;
    }
    else if (lastStableTank != pendingTank && (millis() - tankStabilityTime) >= TANK_STABILITY_THRESHOLD)
    {
        if (pendingTank < 1 || pendingTank > 12)
        {
            tankException = true;
            // Only print on change to avoid flooding serial
            DBG("Wrong Tank ID: ");
            DBGLN(currentRead);
            return;
        }
        lastStableTank = pendingTank;
        tankStabilityTime = 0;
        tankChanged = true;
        DBGLN("Tank Changed & Stable");
    }
    tankException = false;
}

struct DelayedSensor
{
    const uint8_t pin;
    unsigned long lastActiveTime;
    bool stableActive; // Added to store the "last known" state
    const unsigned long delay;

    void update()
    {
        bool rawReading = (sensorActive(pin));

        if (rawReading)
        {
            if (lastActiveTime == 0)
                lastActiveTime = millis();

            if (!stableActive && millis() - lastActiveTime >= delay)
                stableActive = true;
        }
        else
            reset();
    }

    inline bool isActive() { return stableActive; }

    void reset()
    {
        stableActive = false;
        lastActiveTime = 0;
    }
};

// Initialize your specific sensors
DelayedSensor topLimit = {SENSOR_TOP, 0};
DelayedSensor bottomLimit = {SENSOR_BOTTOM, 0};
DelayedSensor wax1Ready = {SENSOR_WAX1, 0};
DelayedSensor wax2Ready = {SENSOR_WAX2, 0};
DelayedSensor startButton = {START_BUTTON, 0, false, BUTTON_DELAY_MS};
DelayedSensor skipButton = {SKIP_BUTTON, 0, false, BUTTON_DELAY_MS};
DelayedSensor raiseButton = {RAISE_BUTTON, 0, false, BUTTON_DELAY_MS};

struct TankProfile
{
    uint8_t dwellMinutes;
    uint8_t requiredHeater; // 0: None, 1: Heater1, 2: Heater2, 3: Both
    uint8_t requiredWax;    // 0: None, 1: Wax1, 2: Wax2, 3: Both
    uint8_t cycles;         // 1: Normal, 2: Double dwell (for tanks 11, 12)
} __attribute__((packed));

// Now define your 12 tanks in one clean table
const TankProfile tanks[13] PROGMEM =
    {
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

// Accessors for PROGMEM table
static inline uint8_t getDwellMinutes(uint8_t idx)
{
    return pgm_read_byte(&(tanks[idx].dwellMinutes));
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
    // wrap explicitly: next after 12 -> 1 (or return 0 if you prefer "no next")
    uint8_t next = (idx >= 12) ? 1 : (idx + 1);
    return (uint8_t)pgm_read_byte(&(tanks[next].requiredWax));
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
    motorStartTime = millis();

    DBGLN("Moving on");
}
void moveOff()
{
    if (!isMoving)
        return;

    digitalWrite(MOVE_PIN, LOW);
    isMoving = false;
    motorStartTime = 0;

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
    if (!isHeating1)
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
void lcdPrintPadded(
    const __FlashStringHelper
        *text)
{ // 16 characters + 1 for the null terminator '\0'
    char lcdBuffer[17];

    // Fill with spaces and terminate
    memset(lcdBuffer, ' ', 16);
    lcdBuffer[16] = '\0';

    // Safely get the flash pointer
    PGM_P p = reinterpret_cast<PGM_P>(text);

    // Measure length in Flash
    size_t len = strnlen_P(p, 16);

    // Copy from Flash to RAM buffer
    memcpy_P(lcdBuffer, p, len);

    // Send the clean, padded 16-char string to LCD
    lcd.print(lcdBuffer);

    DBGLN(text);
}

// Helper for RAM strings
void lcdPrintPadded(const char *text)
{
    // 16 characters + 1 for the null terminator '\0'
    char lcdBuffer[17];

    // 1. Initialize with spaces and null terminator
    memset(lcdBuffer, ' ', 16);
    lcdBuffer[16] = '\0';

    // 2. Use standard RAM-based string functions
    unsigned int len = strnlen(text, 16);

    // 3. Copy from RAM to our local buffer
    memcpy(lcdBuffer, text, len);

    // 4. Send to LCD
    lcd.print(lcdBuffer);

    DBGLN(text);
}
// Version for F() macro strings (Flash memory)
void lcdShowStatus(const __FlashStringHelper *line1,
                   const __FlashStringHelper *line2)
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
    char buffer[17]; // 16 chars + null terminator

    // 1. Fill prefix
    memcpy(buffer, "Tank: ", 6);

    // 2. Handle Tank Number (01-12)
    // If you want " 1" instead of "01", replace '0' with (tank < 10 ? ' ' : '0')
    buffer[6] = (tank / 10 == 0) ? ' ' : ('0' + (tank / 10));
    buffer[7] = '0' + (tank % 10);

    // 3. Fill the remaining 8 spaces to clear the line
    // Index 8 to 15 (8 characters)
    memset(&buffer[8], ' ', 8);

    // 4. Null terminator
    buffer[16] = '\0';

    lcd.print(buffer);

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

void formatTime(char *buf, uint8_t h, uint8_t m, uint8_t s)
{
    // Logic: '0' + (digit) converts a number to its ASCII character
    memcpy(buf, "Time: ", 6);
    buf[6] = '0' + (h / 10);
    buf[7] = '0' + (h % 10);
    buf[8] = ':';
    buf[9] = '0' + (m / 10);
    buf[10] = '0' + (m % 10);
    buf[11] = ':';
    buf[12] = '0' + (s / 10);
    buf[13] = '0' + (s % 10);

    // Fill the rest of the 16-char LCD line
    buf[14] = ' ';
    buf[15] = ' ';
    buf[16] = '\0'; // Null terminator
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
    uint8_t totalMins =
        (tank >= 1 && tank <= 12) ? getDwellMinutes(tank) : MIN_DWELL_MIN;

    // 3. Calculate Elapsed and Remaining
    unsigned long totalMs = (unsigned long)totalMins * ONE_MIN_MS;
    unsigned long elapsedMs = millis() - startTimeTank;
    // Use long to prevent underflow wrap-around errors
    long remainingMs = (long)totalMs - (long)elapsedMs;
    if (remainingMs < 0)
        remainingMs = 0;

    // 4. Conversion
    unsigned long remainingTotalMins = remainingMs / ONE_MIN_MS;
    unsigned int dispalySeconds = (remainingMs / 1000UL) % 60;
    unsigned int hours = remainingTotalMins / 60;
    unsigned int displayMins = remainingTotalMins % 60;

    // 5. LCD Update
    lcd.setCursor(0, 0);
    LcdShowTank(tank);

    // 5. LCD Update (Optimized)
    char timeBuffer[17];
    // Format: "Tank: 12  01:30:05"
    formatTime(timeBuffer, hours, displayMins, dispalySeconds);

    lcd.setCursor(0, 1);
    lcd.print(timeBuffer);
}

// FSM state enumeration
enum MainState : id_t
{
    S_VERIFYING = 0,
    S_UNKNOWN_DIRECTION_RECOVERY,
    S_MIDDLE_RECOVERY,
    S_UP_RECOVERY,
    S_IDLE,
    S_PRE_DOWN,
    S_DOWN,
    S_CHECKING,
    S_PRE_RAISING,
    S_RAISING,
    S_UP,
    S_TRANSITIONING,
    S_STARTING_NEW_TANK,
    S_LOWERING,
    S_ERROR
};

// Forward declarations for predicate/process/event functions
bool verifyingPredicate(id_t id);
void verifyingActionChanged(EventArgs e);

bool unknownDirectionPredicate(id_t id);
void unknownDirectionActionChanged(EventArgs e);

bool middlePredicate(id_t id);
void middleActionChanged(EventArgs e);

bool upRecoveryPredicate(id_t id);
void upRecoveryActionChanged(EventArgs e);

bool idlePredicate(id_t id);
void idleActionChanged(EventArgs e);

bool checkingPredicate(id_t id);
void checkingActionChanged(EventArgs e);

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

void preDownActionChanged(EventArgs e);

void preRaisingActionChanged(EventArgs e);

// Transition table - keep it as readable blocks. Use predicate timers where
// necessary.
Transition transitions[] = {
    // S_VERIFYING: if sample is down -> continue as correct behavior
    //              otherwise enter recovery mode
    {verifyingPredicate, S_IDLE, S_UNKNOWN_DIRECTION_RECOVERY, nullptr, verifyingActionChanged, VERIFICATION_DELAY_MS, PREDIC_TIMER},

    // S_UNKNOWN_DIRECTION_RECOVERY: if sample up -> go to S_UP_RECOVERY
    //                              if not top and not down, so sample is on the middle position
    {unknownDirectionPredicate, S_UP_RECOVERY, S_MIDDLE_RECOVERY, nullptr, unknownDirectionActionChanged},

    // S_MIDDLE_RECOVERY: if sample moved to new tank -> S_STARTING_NEW_TANK
    //              otherwise start lowing in same tank
    {middlePredicate, S_MIDDLE_RECOVERY, S_UP, nullptr, middleActionChanged},

    // S_UP_RECOVERY: if sample moved to new tank -> S_STARTING_NEW_TANK
    //              otherwise start lowing in same tank
    {upRecoveryPredicate, S_STARTING_NEW_TANK, S_LOWERING, nullptr, upRecoveryActionChanged},

    // S_IDLE: wait start button. When pressed -> CHECK_START
    {idlePredicate, S_IDLE, S_PRE_DOWN, nullptr, idleActionChanged},

    // S_PRE_DOWN: Just wait for MOTOR_SWITCH_DELAY_MS before moving to next
    // state
    // (safe switching between moving and vibrating)
    {nullptr, S_PRE_DOWN, S_DOWN, nullptr, preDownActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    /*S_DOWN: Vibrating for 1 hour
            Conatiner 10 -> Start first heater.
            Container 11 -> Start second heater.
            Container 12 -> If finished then stop vibrating.
            */
    {downPredicate, S_PRE_RAISING, S_CHECKING, downProcess, downActionChanged, TANK_TIME_MS, TRUE_TIMER},

    /*S_CHECKING: Container 1..10 -> continue to raise state
            Conatiner 11 + 12 -> Two hours instead of 1 hour, so renter the downstate.
            Conatiner 10 -> Renter if first wax sensor is not ready.
            Conatiner 11 -> Renter if first wax sensor is not ready.
            */
    {checkingPredicate, S_DOWN, S_PRE_RAISING, nullptr, checkingActionChanged},

    // S_PRE_RAISING: Just wait for MOTOR_SWITCH_DELAY_MS before moving to next state
    // (safe switching between moving and vibrating)
    {nullptr, S_PRE_RAISING, S_RAISING, nullptr, preRaisingActionChanged, MOTOR_SWITCH_DELAY_MS, TRANS_TIMER},

    // S_RAISING: run movement up until top sensor active OR timeout -> TOP or ERROR
    {raisingPredicate, S_RAISING, S_UP, nullptr, raisingActionChanged},

    // S_UP: wait if insepction is acitvitated, otherwise proceed to transition state
    {upPredicate, S_UP, S_TRANSITIONING, upProcess, upActionChanged},

    // S_TRANSITIONING: small delay between tanks, then either go to next tank's
    // logic or to START if finished
    {transitiningPredicate, S_TRANSITIONING, S_STARTING_NEW_TANK, nullptr, transitiningActionChanged},

    // S_STARTING_NEW_TANK: read tank and prepare moving down the sample
    {startingPredicate, S_STARTING_NEW_TANK, S_LOWERING, nullptr, startingActionChanged},

    // S_LOWERING: run movement motor until bottom sensor active -> if bottom
    // sensor active -> DOWN else ERROR
    {loweringPredicate, S_LOWERING, S_PRE_DOWN, nullptr, loweringActionChanged},

    // ERROR (top or down sensors, heat sensors, motor, heaters)
    {nullptr, S_ERROR, S_ERROR, nullptr, errorActionChanged}};

const uint8_t numberOfTransitions = sizeof(transitions) / sizeof(Transition);

FiniteState fsm(transitions, numberOfTransitions);

// Implementation: Predicates, Processes and Events
bool verifyingPredicate(id_t id)
{
    if (bottomLimit.isActive())
        return false;

    return true;
}

void verifyingActionChanged(EventArgs e)
{
    if (e.action == ENTRY)
        syncTankID();
}

bool unknownDirectionPredicate(id_t id)
{
    if (topLimit.isActive())
        return false;

    return true;
}

void unknownDirectionActionChanged(EventArgs e)
{
    if (e.action == ENTRY)
        syncTankID();
}

bool upRecoveryPredicate(id_t id)
{
    if (topLimit.isActive() && tankChanged)
        return false;

    if (!bottomLimit.isActive() && !topLimit.isActive())
        return true;

    syncTankID();
}

void upRecoveryActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        syncTankID();

        if (!isMoving)
            moveOn();

        lcdShowStatusTank(F("Recovery Up")); // Uses F() to keep text in Flash
        break;

    case EXIT:
        DBGLN("Exit Lowering");
        break;
    }
}

bool middlePredicate(id_t id)
{
    if (bottomLimit.isActive())
        fsm.begin(S_PRE_DOWN);

    if (topLimit.isActive())
        return true;

    return false;
}

void middleActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        syncTankID();

        if (!isMoving)
            moveOn();

        lcdShowStatusTank(F("Recovery Middle")); // Uses F() to keep text in Flash
        break;

    case EXIT:
        DBGLN("Exit Lowering");
        break;
    }
}

bool idlePredicate(id_t id)
{
    if (startButton.isActive())
    {
        DBGLN("Start button pressed");
        startButton.reset();
        return true;
    }
    if (skipButton.isActive())
    {
        vibOff();
        skipButton.reset();
        DBGLN("Raising to top");
        lcdShowStatus(F("Skip tank"), F("Raising..."));
        fsm.begin(S_RAISING);
    }

    return false;
}

void idleActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        outputsKill();

        syncTankID();
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
        return true;

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

    return false;
}

void startingActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
    {
        syncTankID();

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
    if (bottomLimit.isActive() && finished)
        fsm.begin(S_IDLE);

    // if bottom sensor active -> true, so move to DOWN
    if (bottomLimit.isActive())
    {
        moveOff();
        DBGLN("Reached bottom");
        return true;
    }

    return false; // continue lowering
}
void loweringActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        if (!isMoving)
            moveOn();

        lcdShowStatusTank(F("Lowering..")); // Uses F() to keep text in Flash
        break;

    case EXIT:
        DBGLN("Exit Lowering");
        break;
    }
}

bool downPredicate(id_t id)
{
    if (!bottomLimit.isActive())
    {
        lcdShowStatus(F("ERROR"), F("TOP or BOTTOM S"));
        fsm.begin(S_ERROR);
    }

    if (skipButton.isActive() || raiseButton.isActive())
    {
        vibOff();
        inspection = raiseButton.isActive();
        raiseButton.reset();
        skipButton.reset();
        DBGLN("Raising to top");
        lcdShowStatus(F("Button Pressed"), F("Raising..."));
        return false;
    }
    if (startButton.isActive())
    {
        vibOff();
        moveOff();
        startButton.reset();
        lcdShowStatus(F("Button pressed"), F("Go to idle"));
        fsm.begin(S_IDLE);
    }
    if (waitingWaxMelt)
    {
        uint8_t s = getNextRequiredSensor(lastStableTank);
        if ((s & 1) && !wax1Ready.isActive())
            return true;

        if ((s & 2) && !wax2Ready.isActive())
            return true;

        waitingWaxMelt = false;
        return false;
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

    if (raiseButton.isActive())
    {
        inspection = false;
        raiseButton.reset();
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
    if (topLimit.isActive())
    {
        DBGLN("Reached Top");
        return true;
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
            moveOn();

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

    syncTankID();

    return tankChanged;
}
void transitiningActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        // check mechanical timeout
        if (!isMoving)
            moveOn();

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

void preRaisingActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        vibOff();
        break;

    case EXIT:
        break;
    }
}

void preDownActionChanged(EventArgs e)
{
    switch (e.action)
    {
    case ENTRY:
        moveOff();
        break;

    case EXIT:
        startTimeTank = 0;
        break;
    }
}
// Setup and loop
void handleSensorsFailure()
{
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
void buttonsTask()
{
    startButton.update();
    skipButton.update();
    raiseButton.update();
}

void safetyTask()
{
    if (fsm.id == S_ERROR)
        return;

    if (topLimit.isActive() && bottomLimit.isActive())
    {
        lcdShowStatus(F("ERROR"), F("TOP or BOTTOM S"));
        fsm.begin(S_ERROR);
    }

    if (tankException)
    {
        lcdShowStatus(F("ERROR"), F("Tank read"));
        fsm.begin(S_ERROR);
    }
    if (motorStartTime && (millis() - motorStartTime > MOVE_TIMEOUT_MS))
    {
        // timeout -> error
        moveOff();
        DBGLN("Motor timeout -> ERROR");
        lcdShowStatus(F("ERROR"), F("MOTOR OVER TIME"));
        fsm.begin(S_ERROR);
    }
}

void fsmTask() { fsm.execute(); }

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
    pinMode(SKIP_BUTTON, INPUT_PULLUP);
    pinMode(RAISE_BUTTON, INPUT_PULLUP);

    for (uint8_t i = 0; i < 4; i++)
        pinMode(PIN_ID_BITS[i], INPUT);
}

void setup()
{
#ifdef DEBUG
    Serial.begin(115200);
    while (!Serial)
        ;
#endif
    setupPins();
    Wire.begin();
    lcd.init();
    lcd.backlight();
    syncTankID();
    fsm.begin(S_VERIFYING);
    wdt_enable(WDTO_2S);
}

void loop()
{
    unsigned long now = millis();
    if (now - lastTick >= TICK_MS)
    {

#ifdef DEBUG
        unsigned long start = micros(); // 🔹 start timing
#endif

        lastTick = now;

        wdt_reset();
        sensorTask();
        buttonsTask();
        safetyTask();
        fsmTask();

#ifdef DEBUG
        unsigned long duration = micros() - start; // 🔹 end timing
        if (duration > 1000)
        {
            DBG("Over Time: ");
            DBGLN(duration); // 🔹 print µs
        }
#endif
    }
}