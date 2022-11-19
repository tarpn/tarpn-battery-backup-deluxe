from typing import Iterator


def backoff(start_time, growth_factor, max_time) -> Iterator[float]:
    """Infinite iterator of exponentially increasing backoff times"""
    yield start_time
    next_time = start_time
    while True:
        next_time = min(max_time, next_time * growth_factor)
        yield next_time


class BackoffGenerator(Iterator):
    def __init__(self, initial_wait: float, growth_factor: float, max_wait: float):
        self.initial_wait = initial_wait
        self.growth_factor = growth_factor
        self.max_wait = max_wait
        self.next_wait = initial_wait * growth_factor
        self._total_waited = 0.0
        self._count = 0

    def __next__(self):
        this_wait = self.next_wait
        self.next_wait = min(self.max_wait, self.next_wait * self.growth_factor)
        self._total_waited += this_wait
        self._count += 1
        return this_wait

    def total(self):
        return self._total_waited

    def reset(self) -> bool:
        """
        Return true if this generator had been invoked since the last reset
        """
        self.next_wait = self.initial_wait
        self._total_waited = 0.0
        count = self._count
        self._count = 0
        return count > 0

    def __repr__(self):
        return f"BackoffGenerator(next_wait={self.next_wait}, total_wait={self._total_waited})"
