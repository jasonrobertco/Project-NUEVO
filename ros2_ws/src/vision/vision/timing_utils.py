from __future__ import annotations

import time


class FixedRateScheduler:
    """Keep a loop near a target rate without trying to catch up after overruns."""

    def __init__(self, hz: float) -> None:
        self.period = 1.0 / float(hz) if hz > 0.0 else 0.0
        self.next_time = time.monotonic()

    def wait_until_ready(self) -> None:
        if self.period <= 0.0:
            return

        sleep_time = self.next_time - time.monotonic()
        if sleep_time > 0.0:
            time.sleep(sleep_time)

    def schedule_next(self) -> None:
        if self.period <= 0.0:
            return

        self.next_time += self.period
        if self.next_time < time.monotonic():
            self.reset()

    def reset(self) -> None:
        self.next_time = time.monotonic()
