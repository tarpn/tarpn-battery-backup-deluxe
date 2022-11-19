import threading
from typing import Optional


class CountDownLatch:
    def __init__(self, count=1):
        self._count = count
        self.lock = threading.Condition()

    def count(self) -> int:
        with self.lock:
            return self._count

    def countdown(self):
        self.lock.acquire()
        self._count -= 1
        if self._count <= 0:
            self.lock.notifyAll()
        self.lock.release()

    def join(self, timeout: Optional[float] = None):
        self.lock.acquire()
        while self._count > 0:
            if not self.lock.wait(timeout):
                break
        self.lock.release()

    def cancel(self):
        self.lock.acquire()
        self._count = 0
        self.lock.notifyAll()
        self.lock.release()