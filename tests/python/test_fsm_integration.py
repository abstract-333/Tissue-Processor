# tests/python/test_fsm_integration.py
import pytest
from collections import deque

# We'll implement a tiny FSM and a simulated environment with pins & millis.

class SimPins:
    def __init__(self):
        self.pin_state = {}
        self.outputs = {}
        self._millis = 0

    def digitalRead(self, pin):
        return 0 if self.pin_state.get(pin, False) else 1

    def digitalWrite(self, pin, val):
        self.outputs[pin] = val

    def advance(self, ms):
        self._millis += ms

    def millis(self):
        return self._millis


# A _very_ compact version of the key behavior (only enough to exercise flow)
class SimpleController:
    S_IDLE, S_LOWERING, S_DOWN, S_RAISING, S_UP, S_ERROR = range(6)

    def __init__(self, pins):
        self.p = pins
        self.state = self.S_IDLE
        self.motor_on = False
        self.vib_on = False
        self.start_time = None

    def start_button(self):
        return self.p.digitalRead('BTN') == 0

    def tick(self):
        if self.state == self.S_IDLE:
            if self.start_button():
                self.state = self.S_LOWERING
                self.motor_on = True
        elif self.state == self.S_LOWERING:
            # bottom sensor active -> down
            if self.p.digitalRead('BOTTOM') == 0:
                self.motor_on = False
                self.state = self.S_DOWN
                self.vib_on = True
                self.start_time = self.p.millis()
        elif self.state == self.S_DOWN:
            # simulate one second down time as TANK_TIME_MS
            if self.p.millis() - self.start_time >= 1000:
                self.vib_on = False
                self.state = self.S_RAISING
                self.motor_on = True
        elif self.state == self.S_RAISING:
            if self.p.digitalRead('TOP') == 0:
                self.motor_on = False
                self.state = self.S_UP
