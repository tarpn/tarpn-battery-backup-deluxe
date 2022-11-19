import threading
from typing import Callable
import time


class Timer:
    def __init__(self, delay_ms: float, cb: Callable[[], None]):
        """
        A resettable and cancelable timer class
        :param delay_ms: Delay in milliseconds
        :param cb: A callback that takes no arguments
        """
        self.delay = delay_ms
        self._cb = cb
        self._started = 0
        self._timer = None

    def __repr__(self):
        return f"Timer(delay={self.delay}, remaining={self.remaining()})"

    def start(self):
        if self._timer:
            self._timer.cancel()
        self._started = time.time()
        self._timer = threading.Timer(self.delay / 1000., self._run_cb)
        self._timer.start()

    def _run_cb(self):
        if self._timer:
            # Callback might restart the timer, so clear the timer object first
            self._timer = None
            self._cb()

    def cancel(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def reset(self):
        self.cancel()
        self.start()

    def running(self):
        return self._timer is not None

    def remaining(self):
        if self.running():
            return self.delay - ((time.time() - self._started) * 1000.)
        else:
            return -1
