#include "TissueProcessor.ino" // include your firmware headers (with UNIT_TEST hooks)
#include <unity.h>

// We'll implement mocks for digitalRead/digitalWrite/millis and provide helpers
static uint32_t fake_millis = 0;
static int fake_pin_state[256];
static int fake_pin_write[256];

extern "C" uint32_t millis() { return fake_millis; }
extern "C" int digitalRead(int pin) { return fake_pin_state[pin]; }
extern "C" void digitalWrite(int pin, int val) { fake_pin_write[pin] = val; }

void setPinLow(int pin) { fake_pin_state[pin] = 0; }
void setPinHigh(int pin) { fake_pin_state[pin] = 1; }
void advanceMs(uint32_t ms) { fake_millis += ms; }

void setUp(void)
{
  // initialize your firmware with UNIT_TEST flag path
  firmware_setup_for_test();
}

void tearDown(void)
{
  // reset arrays
}

void test_sequence_start_to_down(void)
{
  setPinHigh(START_BUTTON); // not pressed (INPUT_PULLUP -> HIGH)
  // simulate press: set to LOW
  setPinLow(START_BUTTON);
  loop_tick_sim(); // one scheduler tick
  TEST_ASSERT_EQUAL(S_LOWERING, fsm_id_for_test());

  // bottom sensor low -> reached bottom
  setPinLow(SENSOR_BOTTOM);
  advanceMs(20);
  loop_tick_sim();
  TEST_ASSERT_EQUAL(S_DOWN, fsm_id_for_test());
  TEST_ASSERT_EQUAL(1, fake_pin_write[VIB_PIN]); // vibrating on
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_sequence_start_to_down);
  return UNITY_END();
}