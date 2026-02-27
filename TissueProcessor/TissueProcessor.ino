#include "Main.h"
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial)
    ;
#endif
  setupPins();
  Wire.begin();
  lcd.init();
  lcd.backlight();
  readTankID();
  fsm.begin(S_IDLE);
  wdt_enable(WDTO_2S);
}

void loop() {
  unsigned long now = millis();
  if (now - lastTick >= TICK_MS) {

#ifdef DEBUG
    unsigned long start = micros(); // 🔹 start timing
#endif

    lastTick = now;

    wdt_reset();
    sensorTask();
    safetyTask();
    fsmTask();

#ifdef DEBUG
    unsigned long duration = micros() - start; // 🔹 end timing
    if (duration > 1000) {
      DBG("Over Time: ");
      DBGLN(duration); // 🔹 print µs
    }
#endif
  }
}
