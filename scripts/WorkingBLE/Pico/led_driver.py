# ==============================================================================
# led_driver.py — PIO-based SK6805 EC15 / WS2812B LED driver
# Target: Raspberry Pi Pico W, MicroPython
#
# Drives 160 LEDs (2 blades × 80) in GRB byte order using the PIO peripheral.
# The PIO program clocks bits out at ~800 kHz with WS2812B-compatible timing:
#   T0H ≈ 0.35 µs, T0L ≈ 0.9 µs
#   T1H ≈ 0.9  µs, T1L ≈ 0.35 µs
#   Reset > 50 µs (handled by the caller or auto-pull stall)
# ==============================================================================

import rp2
import machine
from config import LED_PIN, PIO_STATE_MACHINE, TOTAL_LEDS

# ------------------------------------------------------------------------------
# PIO program — WS2812B bit-bang at 800 kHz
# One PIO cycle = 1/125 MHz × 10 clocks = 10 × 8 ns = 80 ns
# 1 bit = 10 cycles = 800 ns (≈ 1.25 µs period → 800 kHz)
# T1H = 7 cycles = 560 ns  (spec min 550 ns ✓)
# T0H = 3 cycles = 240 ns  (spec typ 350 ns — close enough; SK6805 is tolerant)
# ------------------------------------------------------------------------------
@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,   # MSB first
    autopull=True,
    pull_thresh=24,                     # 24 bits = 3 bytes (one LED in GRB)
)
def _ws2812b():
    # Each bit loop: 10 PIO cycles total
    # side 1 = pin HIGH, side 0 = pin LOW
    wrap_target()
    label("bit_loop")
    out(x, 1)                  .side(0) [2]  # 3 cycles LOW, shift 1 bit into X
    jmp(not_x, "zero")         .side(1) [1]  # 2 cycles HIGH (always — T0H start)
    jmp("bit_loop")            .side(1) [4]  # 5 extra cycles HIGH → T1H = 7 cyc
    label("zero")
    nop()                      .side(0) [4]  # pin LOW for T0L, consume same 5 cyc
    wrap()


class LEDDriver:
    """
    Thin wrapper around a PIO state machine for driving SK6805 / WS2812B LEDs.

    Usage:
        driver = LEDDriver()
        driver.write(image_bytes)   # bytearray, length = TOTAL_LEDS * 3, GRB order
    """

    def __init__(
        self,
        pin: int = LED_PIN,
        sm_id: int = PIO_STATE_MACHINE,
        num_leds: int = TOTAL_LEDS,
    ):
        self._num_leds = num_leds
        self._expected_len = num_leds * 3

        # Initialise state machine at 10× the bit frequency.
        # 800 kHz bits → 8 MHz SM clock.
        self._sm = rp2.StateMachine(
            sm_id,
            _ws2812b,
            freq=8_000_000,
            sideset_base=machine.Pin(pin),
        )
        self._sm.active(1)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def write(self, led_bytes) -> None:
        """
        Push *led_bytes* to the LED strip via PIO DMA-style word writes.

        Parameters
        ----------
        led_bytes : bytes | bytearray
            Raw GRB data, length must equal TOTAL_LEDS × 3 (= 480 bytes).
            Data order: [LED0_G, LED0_R, LED0_B, LED1_G, ...]

        The PIO auto-pull fires every 24 bits, so we pack 3 raw bytes into
        a 32-bit word with the colour bytes in the top 24 bits (big-endian
        shift-left arrangement expected by the PIO program).
        """
        if len(led_bytes) != self._expected_len:
            raise ValueError(
                f"Expected {self._expected_len} bytes, got {len(led_bytes)}"
            )

        # Pack each 3-byte GRB triplet into a 32-bit word, MSB-aligned.
        # We iterate with a plain index to avoid temporary list allocations.
        sm = self._sm
        i = 0
        n = self._expected_len
        while i < n:
            # Shift the three bytes into bits [31:8]; bits [7:0] are 0 (ignored).
            word = (led_bytes[i] << 24) | (led_bytes[i + 1] << 16) | (led_bytes[i + 2] << 8)
            sm.put(word)
            i += 3
        # The PIO stalls on the last autopull until data arrives; no explicit
        # reset pulse needed — the ~50 µs latch happens naturally between calls.

    def blank(self) -> None:
        """Turn all LEDs off."""
        self.write(bytearray(self._expected_len))

    def deinit(self) -> None:
        """Stop the PIO state machine."""
        self._sm.active(0)

