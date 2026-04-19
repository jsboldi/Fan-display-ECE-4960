# ==============================================================================
# display_controller.py — IR-synced POV display loop
# Target: Raspberry Pi Pico W, MicroPython
#
# No changes from original — the _slice_start = now timing logic is correct
# once led_driver.py's sequential-write bug is fixed.  With parallel writes
# (~2,400 µs), the 16-slice window (4,167 µs) is wide enough that capturing
# 'now' before the write works fine: after one 200 µs yield the elapsed time
# is still below threshold, so the next write fires ~33 µs late (0.8% error).
#
# Modes:
#   FREERUN  — no IR signal: cycles slices on a fixed timer at 900 RPM
#   SYNCED   — IR signal present: cycles based on measured revolution period
# ==============================================================================

import machine
import utime
from config import (
    SLICES, SLICE_STRIDE, IR_PIN, IR_TIMEOUT_MS,
)

FREERUN_RPM       = 900
FREERUN_PERIOD_US = int(60_000_000 / FREERUN_RPM)   # 66 667 µs


class DisplayController:

    def __init__(self, led_driver, active_buffer: bytearray):
        self._driver       = led_driver
        self.active_buffer = active_buffer

        self._slice_count = SLICES
        self._slice_us    = FREERUN_PERIOD_US // SLICES

        self._slice_start = utime.ticks_us()
        self._slice_idx   = 0

        self._rev_period_us  = 0
        self._ir_slice_start = 0
        self._ir_slice_idx   = 0
        self._ir_slice_us    = 0
        self._last_ir_us     = 0
        self._ir_active      = False

        self._ir_pin = machine.Pin(IR_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        self._ir_pin.irq(trigger=machine.Pin.IRQ_FALLING,
                         handler=self._ir_callback)

        print("[DISP] Ready — {} slices, {} µs/slice, freerun @ {} RPM".format(
            self._slice_count, self._slice_us, FREERUN_RPM))

    # ------------------------------------------------------------------
    # IR interrupt
    # ------------------------------------------------------------------

    def _ir_callback(self, pin) -> None:
        now = utime.ticks_us()
        if self._last_ir_us != 0:
            period = utime.ticks_diff(now, self._last_ir_us)
            if 10_000 < period < 2_000_000:
                self._rev_period_us = period
                self._ir_slice_us   = period // self._slice_count
                self._ir_active     = True
        self._last_ir_us     = now
        self._ir_slice_start = now
        self._ir_slice_idx   = 0

    # ------------------------------------------------------------------
    # Main-loop API
    # ------------------------------------------------------------------

    def step(self) -> None:
        """Non-blocking — call as fast as possible from main loop."""
        now = utime.ticks_us()

        ir_idle = (self._last_ir_us == 0 or
                   utime.ticks_diff(now, self._last_ir_us) // 1000 > IR_TIMEOUT_MS)

        if ir_idle or not self._ir_active:
            self._step_freerun(now)
        else:
            self._step_synced(now)

    def _step_freerun(self, now) -> None:
        if utime.ticks_diff(now, self._slice_start) < self._slice_us:
            return
        self._write_slice(self._slice_idx)
        self._slice_idx   = (self._slice_idx + 1) % self._slice_count
        self._slice_start = now

    def _step_synced(self, now) -> None:
        if self._ir_slice_idx >= self._slice_count:
            return
        elapsed = utime.ticks_diff(now, self._ir_slice_start)
        due     = self._ir_slice_idx * self._ir_slice_us
        if elapsed < due:
            return
        self._write_slice(self._ir_slice_idx)
        self._ir_slice_idx += 1

    def _write_slice(self, idx: int) -> None:
        offset = idx * SLICE_STRIDE
        self._driver.write(
            memoryview(self.active_buffer)[offset : offset + SLICE_STRIDE]
        )

    # ------------------------------------------------------------------
    # Buffer management
    # ------------------------------------------------------------------

    def set_active_buffer(self, buf: bytearray, image_size: int = 0) -> None:
        self.active_buffer = buf

        if image_size > 0:
            slices = image_size // SLICE_STRIDE
            if slices > 0:
                self._slice_count = slices
                self._slice_us    = FREERUN_PERIOD_US // slices
                if self._ir_active and self._rev_period_us > 0:
                    self._ir_slice_us = self._rev_period_us // slices

                self._slice_idx   = 0
                self._slice_start = utime.ticks_us()
                self._ir_slice_idx = 0

                print("[DISP] {} slices, {} µs/slice".format(slices, self._slice_us))

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def rpm(self) -> float:
        if self._rev_period_us == 0:
            return 0.0
        return 60_000_000 / self._rev_period_us

    @property
    def mode(self) -> str:
        return "synced" if self._ir_active else "freerun"

    @property
    def slice_count(self) -> int:
        return self._slice_count
