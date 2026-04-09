# ==============================================================================
# display_controller.py — IR-synced POV display loop
# Target: Raspberry Pi Pico W, MicroPython
#
# Architecture
# ------------
# The IR sensor fires an interrupt on every full revolution.  The interrupt
# handler records the revolution period (µs) and resets a slice counter.
# The main display loop asks "is it time to show the next slice?" by comparing
# elapsed time against (period / SLICES), then calls led_driver.write().
#
# Buffer layout reminder (from config.py):
#   active_buffer[slice_index * SLICE_STRIDE  :  (slice_index+1) * SLICE_STRIDE]
#   = blade0 LEDs (80×3 bytes GRB) ++ blade1 LEDs (80×3 bytes GRB)
#
# Synchronisation contract
# ------------------------
# - ir_callback()  → runs in interrupt context; MUST be short.
# - step()         → called from the main loop; does the heavy lifting.
# - The only shared state between ISR and main loop is:
#     _rev_period_us  (int, written atomically by ISR)
#     _slice_start_us (int, written atomically by ISR on revolution start)
#     _slice_idx      (int, reset to 0 by ISR; incremented by step())
#   MicroPython GIL ensures single-threaded access on Pico; no mutex needed.
# ==============================================================================

import machine
import utime
from config import (
    SLICES, SLICE_STRIDE, IR_PIN, IR_TIMEOUT_MS, TOTAL_LEDS,
)


class DisplayController:
    """
    Drives the LED strip in sync with fan rotation.

    Parameters
    ----------
    led_driver : LEDDriver
        Initialised LEDDriver instance.
    active_buffer : bytearray
        15360-byte image buffer; populated externally by BLEImageReceiver.
    """

    def __init__(self, led_driver, active_buffer: bytearray):
        self._driver        = led_driver
        self.active_buffer  = active_buffer   # shared with BLEImageReceiver

        # --- Rotation state (written by ISR, read by main loop) ---
        self._rev_period_us  = 0    # duration of the last full revolution in µs
        self._slice_start_us = 0    # timestamp (µs) of revolution start (IR pulse)
        self._slice_idx      = 0    # which slice we should display next
        self._last_ir_us     = 0    # timestamp of previous IR pulse (for period calc)

        # Slice duration cache — updated every revolution to avoid division in ISR
        self._slice_period_us = 0

        # --- Blank framebuffer for when fan is idle ---
        self._blank = bytearray(TOTAL_LEDS * 3)

        # --- IR sensor setup ---
        self._ir_pin = machine.Pin(IR_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        # Trigger on falling edge (active-low sensor)
        self._ir_pin.irq(trigger=machine.Pin.IRQ_FALLING,
                         handler=self._ir_callback)

    # ------------------------------------------------------------------
    # ISR — keep it minimal
    # ------------------------------------------------------------------

    def _ir_callback(self, pin) -> None:
        """
        Called on every rising edge of the IR/Hall sensor (one per revolution).
        Updates the revolution period and resets the slice counter.
        """
        now = utime.ticks_us()

        if self._last_ir_us != 0:
            period = utime.ticks_diff(now, self._last_ir_us)
            if period > 0:
                self._rev_period_us  = period
                # Pre-compute slice period to avoid division in the hot loop
                self._slice_period_us = period // SLICES

        self._last_ir_us     = now
        self._slice_start_us = now
        self._slice_idx      = 0   # restart slices from the beginning

    # ------------------------------------------------------------------
    # Main-loop API
    # ------------------------------------------------------------------

    def step(self) -> None:
        """
        Call this as fast as possible from the main loop.
        It decides whether it is time to latch the next slice and, if so,
        drives the LED strip with the appropriate data.

        This function NEVER blocks — it returns immediately if it is not yet
        time for the next slice.
        """
        now = utime.ticks_us()

        # ── Fan-idle guard ───────────────────────────────────────────────────
        # If no IR pulse has arrived for IR_TIMEOUT_MS, blank the strip.
        if self._last_ir_us == 0:
            return   # No revolution seen yet since boot

        idle_ms = utime.ticks_diff(now, self._last_ir_us) // 1000
        if idle_ms > IR_TIMEOUT_MS:
            # Fan has stopped or stalled; blank strip once and return.
            if self._slice_idx != SLICES:  # avoid repeated writes
                self._driver.write(self._blank)
                self._slice_idx = SLICES   # sentinel: "already blanked"
            return

        # ── Slice timing check ───────────────────────────────────────────────
        if self._slice_period_us == 0:
            return   # Period not yet known (first revolution incomplete)

        slice_idx = self._slice_idx
        if slice_idx >= SLICES:
            return   # All slices for this revolution already shown

        # How long since the start of this revolution?
        elapsed = utime.ticks_diff(now, self._slice_start_us)
        # Time at which slice_idx *should* start
        due_us  = slice_idx * self._slice_period_us

        if elapsed < due_us:
            return   # Not yet time for this slice

        # ── Output the slice ─────────────────────────────────────────────────
        offset = slice_idx * SLICE_STRIDE
        # Pass a memoryview to avoid copying 480 bytes — write() accepts it.
        self._driver.write(memoryview(self.active_buffer)[offset : offset + SLICE_STRIDE])

        self._slice_idx = slice_idx + 1

    # ------------------------------------------------------------------
    # Diagnostic helpers
    # ------------------------------------------------------------------

    @property
    def rpm(self) -> float:
        """Estimated fan speed in RPM based on last measured period."""
        if self._rev_period_us == 0:
            return 0.0
        return 60_000_000 / self._rev_period_us

    @property
    def current_slice(self) -> int:
        return self._slice_idx

    def set_active_buffer(self, buf: bytearray) -> None:
        """
        Point the controller at a new active buffer.
        Call this from the main loop (not from IRQ context) after a buffer swap.
        """
        self.active_buffer = buf

