# APP_NAME: RoboticsTimer
import time
import threading
from typing import Callable, List, Optional


class TimerCallback:
    """Represents a single registered timer callback."""

    def __init__(self, callback: Callable, interval: float, name: str):
        self.callback = callback
        self.interval = interval
        self.name = name
        self.last_called = time.monotonic()
        self.call_count = 0
        self.missed_deadlines = 0


class RoboticsTimer:
    """
    A centralised timer for syncing time across multiple sensors/modules.

    Features:
    - Register multiple callbacks at different Hz rates
    - Monotonic clock (immune to system time changes)
    - Missed deadline detection
    - Thread-safe operation
    - Timestamped tick log for review
    """

    def __init__(self, master_rate_hz: float = 100.0):
        """
        Args:
            master_rate_hz: How fast the master loop ticks (default 100 Hz).
                            All sensor rates must divide evenly into this.
        """
        self.master_rate_hz = master_rate_hz
        self.master_interval = 1.0 / master_rate_hz

        self._callbacks: List[TimerCallback] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._start_time: Optional[float] = None
        self._tick_count = 0
        self._tick_log: List[dict] = []  # stores recent ticks for review
        self._max_log_size = 500

        print(f"[RoboticsTimer] Initialised — master rate: {master_rate_hz} Hz")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def register(self, callback: Callable, rate_hz: float, name: str = "unnamed"):
        """
        Register a sensor or module callback to fire at a given rate.

        Args:
            callback : function to call (no args)
            rate_hz  : how often to call it (must be <= master_rate_hz)
            name     : human-readable label for logs
        """
        if rate_hz > self.master_rate_hz:
            raise ValueError(
                f"Callback '{name}' rate {rate_hz} Hz exceeds "
                f"master rate {self.master_rate_hz} Hz."
            )

        interval = 1.0 / rate_hz
        entry = TimerCallback(callback, interval, name)

        with self._lock:
            self._callbacks.append(entry)

        print(f"[RoboticsTimer] Registered '{name}' at {rate_hz} Hz "
              f"(every {interval*1000:.1f} ms)")

    def start(self):
        """Start the master timer loop in a background thread."""
        if self._running:
            print("[RoboticsTimer] Already running.")
            return

        self._running = True
        self._start_time = time.monotonic()
        self._thread = threading.Thread(target=self._master_loop, daemon=True)
        self._thread.start()
        print("[RoboticsTimer] Timer started.")

    def stop(self):
        """Stop the master timer loop."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        print("[RoboticsTimer] Timer stopped.")
        self._print_summary()

    def elapsed(self) -> float:
        """Return seconds since timer started."""
        if self._start_time is None:
            return 0.0
        return time.monotonic() - self._start_time

    def get_timestamp(self) -> dict:
        """Return a structured timestamp snapshot."""
        now = time.monotonic()
        return {
            "monotonic_s" : round(now, 6),
            "elapsed_s"   : round(self.elapsed(), 6),
            "tick"        : self._tick_count,
        }

    def get_log(self) -> List[dict]:
        """Return the recent tick log for review."""
        with self._lock:
            return list(self._tick_log)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _master_loop(self):
        """Core timing loop — runs at master_rate_hz."""
        next_tick = time.monotonic()

        while self._running:
            now = time.monotonic()

            # Fire any callbacks whose interval has elapsed
            with self._lock:
                for cb in self._callbacks:
                    if now - cb.last_called >= cb.interval:
                        # Missed deadline check
                        overrun = (now - cb.last_called) - cb.interval
                        if overrun > cb.interval * 0.1:  # >10% late
                            cb.missed_deadlines += 1

                        try:
                            cb.callback()
                        except Exception as e:
                            print(f"[RoboticsTimer] ERROR in '{cb.name}': {e}")

                        cb.last_called = now
                        cb.call_count += 1

                # Log this tick
                tick_entry = {
                    "tick"      : self._tick_count,
                    "elapsed_s" : round(self.elapsed(), 6),
                    "timestamp" : round(now, 6),
                }
                self._tick_log.append(tick_entry)
                if len(self._tick_log) > self._max_log_size:
                    self._tick_log.pop(0)

            self._tick_count += 1

            # Precise sleep — accounts for processing time
            next_tick += self.master_interval
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _print_summary(self):
        """Print a summary of all callback stats after stopping."""
        print("\n" + "="*50)
        print(f"  RoboticsTimer Summary  |  Total ticks: {self._tick_count}")
        print(f"  Total elapsed: {self.elapsed():.3f} s")
        print("="*50)
        with self._lock:
            for cb in self._callbacks:
                print(f"  [{cb.name}]")
                print(f"    Calls made      : {cb.call_count}")
                print(f"    Missed deadlines: {cb.missed_deadlines}")
        print("="*50 + "\n")


# ----------------------------------------------------------------------
# Demo — simulating 3 sensors synced to one timer
# ----------------------------------------------------------------------

def lidar_callback():
    ts = timer.get_timestamp()
    print(f"  [LiDAR   ] tick={ts['tick']:>5}  elapsed={ts['elapsed_s']:.3f}s")

def camera_callback():
    ts = timer.get_timestamp()
    print(f"  [Camera  ] tick={ts['tick']:>5}  elapsed={ts['elapsed_s']:.3f}s")

def imu_callback():
    ts = timer.get_timestamp()
    print(f"  [IMU     ] tick={ts['tick']:>5}  elapsed={ts['elapsed_s']:.3f}s")


if __name__ == "__main__":
    # Master loop at 100 Hz
    timer = RoboticsTimer(master_rate_hz=100.0)

    # Register sensors at different rates
    timer.register(lidar_callback,  rate_hz=10.0,  name="LiDAR")
    timer.register(camera_callback, rate_hz=5.0,   name="Camera")
    timer.register(imu_callback,    rate_hz=20.0,  name="IMU")

    timer.start()

    # Run for 2 seconds then stop
    time.sleep(2.0)
    timer.stop()

    # Print last 5 tick log entries
    print("Last 5 tick log entries:")
    for entry in timer.get_log()[-5:]:
        print(f"  {entry}")
