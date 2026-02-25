# tests/python/test_unit.py
import time
import pytest

# Minimal re-implementation of small helpers from your firmware that are pure logic

def format_time_py(h, m, s):
    # returns exactly the 16-char line like formatTime -> "Time: HH:MM:SS  "
    buf = list(' ' * 16)
    s0 = f"Time: {h:02d}:{m:02d}:{s:02d}"
    for i, ch in enumerate(s0):
        buf[i] = ch
    return ''.join(buf)


class DebouncedSensorSim:
    def __init__(self, debounce_ms=20):
        self.debounce_ms = debounce_ms
        self.last_low_time = None
        self.stable_active = False
        self._now = 0
        self._raw = False

    def set_raw(self, active_low):
        # active_low == True means sensor reads LOW (active)
        self._raw = active_low

    def advance(self, ms):
        self._now += ms
        # update logic
        if self._raw:
            if self.last_low_time is None:
                self.last_low_time = self._now
            if self._now - self.last_low_time >= self.debounce_ms:
                self.stable_active = True
        else:
            self.last_low_time = None
            self.stable_active = False

    def is_active(self):
        return self.stable_active


# ------------------ Tests ------------------

def test_format_time():
    out = format_time_py(1, 2, 3)
    assert out.startswith('Time: 01:02:03')
    assert len(out) == 16


def test_debounce_true_after_threshold():
    s = DebouncedSensorSim(debounce_ms=50)
    s.set_raw(True)
    s.advance(49)
    assert not s.is_active()
    s.advance(1)
    assert s.is_active()


