import itertools
import threading
from collections import deque
from typing import Generic, TypeVar, Tuple

T = TypeVar('T')


class RingBuffer(Generic[T]):
    """
    A synchronized ring buffer of a fixed sized.
    """
    def __init__(self, size: int):
        self.ring_buffer = deque(maxlen=size)
        self.take_count = itertools.count()
        self.last_take = next(self.take_count)

        self.lock = threading.RLock()
        self.not_empty_cond: threading.Condition = threading.Condition(self.lock)

    def offer(self, item: T) -> None:
        """
        Offer an item into the queue. If it is full, eject the oldest item to make room for this one
        """
        with self.lock:
            self.ring_buffer.append((item, next(self.take_count)))
            self.not_empty_cond.notify()

    def take(self, timeout: float) -> Tuple[T, int]:
        """
        Take the oldest item off this queue. Return the item as well as how many items were dropped
        since the last call to this method.

        This call will block until an item is available or a timeout occurs
        """
        with self.lock:
            if len(self.ring_buffer) == 0:
                self.not_empty_cond.wait(timeout=timeout)

            if len(self.ring_buffer) > 0:
                item, count = self.ring_buffer.popleft()
                dropped = count - self.last_take - 1
                self.last_take = count
                return item, dropped
            else:
                return None, 0
