# led_driver.py
# PIO-based SK6805 EC15 LED driver for Raspberry Pi Pico W
# Target hardware: 2 blades x 80 LEDs = 160 SK6805 EC15 LEDs (WS2812B-compatible protocol)
# Single data wire driven via PIO state machine

import array
import rp2
from machine import Pin

# ---------------------------------------------------------------------------
# SK6805 EC15 Timing (WS2812B-compatible, at 10MHz PIO clock)
# T0H = 300ns, T0L = 900ns → bit-0
# T1H = 600ns, T1L = 600ns → bit-1
# Reset = >80µs low
#
# PIO runs at 10 MHz → 1 cycle = 100ns
# T0: high for 3 cycles (300ns), low for 9 cycles (900ns)  → 12 cycles/bit
# T1: high for 6 cycles (600ns), low for 6 cycles (600ns)  → 12 cycles/bit
# ---------------------------------------------------------------------------

@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24
)
def sk6805_pio():
    """
    PIO program for SK6805 EC15 / WS2812B protocol.
    Sends 24 bits (GRB order) per LED at 10 MHz PIO clock.
    Each bit = 12 cycles (1.2µs total).
      Bit-0: 3 high + 9 low
      Bit-1: 6 high + 6 low
    """
    wrap_target()
    # --- Bit-1: 6 high + 6 low ---
    label("bit1")
    out(x, 1)               .side(0)    [1]  # pull 1 bit; hold low 2 cycles
    jmp(not_x, "bit0")      .side(1)    [3]  # if bit=0 jump; else hold high 4 cycles
    nop()                   .side(1)    [1]  # hold high 2 more → total 6 high
    jmp("bit1")             .side(0)    [4]  # hold low 5 cycles → total 6 low (1 from next top)
    # --- Bit-0: 3 high + 9 low ---
    label("bit0")
    nop()                   .side(0)    [4]  # hold low 5 cycles → total 9 low (3 already from above + nop)
    wrap()


# ---------------------------------------------------------------------------
# Alternative: clean, well-tested WS2812 PIO adapted for SK6805 EC15
# Uses standard rp2 example timing which works for SK6805 (identical protocol)
# ---------------------------------------------------------------------------

@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24
)
def sk6805_pio_v2():
    """
    Robust PIO program based on MicroPython WS2812 reference.
    PIO clock = 8MHz → 1 tick = 125ns
    T0: high=2 ticks (250ns), low=5 ticks (625ns)  ← within SK6805 spec
    T1: high=5 ticks (625ns), low=2 ticks (250ns)  ← within SK6805 spec
    Total = 7 ticks/bit = ~1.14µs/bit
    """
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [2]  # side-set low, shift 1 bit to x, delay 2
    jmp(not_x, "do_zero")   .side(1)    [1]  # side-set high; if x==0 jump to do_zero
    jmp("bitloop")          .side(1)    [4]  # bit=1: stay high 5 ticks, back to loop
    label("do_zero")
    nop()                   .side(0)    [4]  # bit=0: go low 5 ticks
    wrap()


class SK6805Driver:
    """
    Driver for SK6805 EC15 LEDs using PIO on the Raspberry Pi Pico W.

    Hardware:
      - 2 blades, 80 LEDs per blade
      - LEDs wired in series: blade_0[0..79] → blade_1[0..79]
      - Single GPIO data pin
      - GRB color order (standard for SK6805/WS2812B)

    Usage:
        driver = SK6805Driver(pin=0, num_leds=160)
        driver.set_pixel(0, 255, 0, 0)   # pixel 0 = full red
        driver.show()
    """

    # Physical constants
    LEDS_PER_BLADE = 80
    NUM_BLADES     = 2
    TOTAL_LEDS     = LEDS_PER_BLADE * NUM_BLADES   # 160

    # PIO state machine index (0-7 on Pico)
    SM_ID = 0

    # PIO clock frequency: 8 MHz for sk6805_pio_v2 timing above
    PIO_FREQ = 8_000_000

    # Reset latch time: SK6805 EC15 requires >80µs low; we use 100µs
    RESET_US = 100

    # Default global brightness scale (0–255)
    DEFAULT_BRIGHTNESS = 128

    def __init__(self, pin: int = 0, num_leds: int = TOTAL_LEDS, brightness: int = DEFAULT_BRIGHTNESS):
        """
        Args:
            pin:        GPIO pin number connected to LED data line
            num_leds:   Total number of LEDs in the chain (default 160)
            brightness: Global brightness scale 0–255 (default 128 = 50%)
        """
        self._num_leds  = num_leds
        self._pin       = Pin(pin, Pin.OUT)
        self._brightness = max(0, min(255, brightness))

        # Pixel buffer: stores 32-bit words in GRB format, MSB-aligned for PIO
        # Each entry: 0xGGRRBB00 (top 24 bits used, bottom 8 ignored by PIO)
        self._buf = array.array("I", [0] * num_leds)

        # Initialise PIO state machine
        self._sm = rp2.StateMachine(
            self.SM_ID,
            sk6805_pio_v2,
            freq=self.PIO_FREQ,
            sideset_base=self._pin
        )
        self._sm.active(1)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_pixel(self, index: int, r: int, g: int, b: int) -> None:
        """
        Set a single LED by index (0-based, across both blades).

        Args:
            index: LED index 0..(num_leds-1)
            r, g, b: Color components 0–255 (brightness scaling applied internally)
        """
        if not (0 <= index < self._num_leds):
            return
        r, g, b = self._scale(r), self._scale(g), self._scale(b)
        # PIO expects GRB, MSB-first, packed into upper 24 bits of 32-bit word
        self._buf[index] = (g << 24) | (r << 16) | (b << 8)

    def set_blade(self, blade: int, pixels: list) -> None:
        """
        Set all pixels on one blade at once.

        Args:
            blade:  0 or 1
            pixels: list of (r, g, b) tuples, length = LEDS_PER_BLADE (80)
        """
        if blade not in (0, 1):
            raise ValueError("blade must be 0 or 1")
        offset = blade * self.LEDS_PER_BLADE
        for i, (r, g, b) in enumerate(pixels[:self.LEDS_PER_BLADE]):
            self.set_pixel(offset + i, r, g, b)

    def set_column(self, blade: int, led_index: int, r: int, g: int, b: int) -> None:
        """
        Set a single LED on a specific blade.
        Convenience wrapper matching the display_loop interface.

        Args:
            blade:     0 or 1
            led_index: 0..(LEDS_PER_BLADE-1)
            r, g, b:   Color 0–255
        """
        index = blade * self.LEDS_PER_BLADE + led_index
        self.set_pixel(index, r, g, b)

    def fill(self, r: int, g: int, b: int) -> None:
        """Set all LEDs to the same color."""
        r, g, b = self._scale(r), self._scale(g), self._scale(b)
        word = (g << 24) | (r << 16) | (b << 8)
        for i in range(self._num_leds):
            self._buf[i] = word

    def clear(self) -> None:
        """Turn off all LEDs."""
        for i in range(self._num_leds):
            self._buf[i] = 0

    def show(self) -> None:
        """
        Push pixel buffer to LEDs via PIO.
        Blocks until transmission is complete, then holds reset low.
        Total time: (num_leds × 24 bits × 1.14µs) + 100µs reset
          ≈ 160 × 24 × 1.14µs ≈ 4.4ms + 0.1ms = ~4.5ms
        """
        self._sm.put(self._buf, 8)   # shift by 8: PIO uses top 24 of 32 bits
        # Reset latch: hold data line low for >80µs
        # The PIO leaves the line low after last bit, so a utime delay suffices
        import utime
        utime.sleep_us(self.RESET_US)

    def set_brightness(self, brightness: int) -> None:
        """
        Update global brightness scale (0–255).
        Takes effect on the next set_pixel / fill call.
        Does NOT retroactively update the buffer.
        """
        self._brightness = max(0, min(255, brightness))

    @property
    def num_leds(self) -> int:
        return self._num_leds

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _scale(self, value: int) -> int:
        """Apply global brightness scaling to a single channel value."""
        return (value * self._brightness) >> 8


# ---------------------------------------------------------------------------
# Blade-aware helpers used by display_loop
# ---------------------------------------------------------------------------

def make_slice_from_rgb_bytes(driver: SK6805Driver, blade: int, rgb_bytes: bytes) -> None:
    """
    Write one angular slice (80 LEDs) to a blade directly from raw RGB bytes.

    Args:
        driver:    SK6805Driver instance
        blade:     0 or 1
        rgb_bytes: bytes object of length 240 (80 LEDs × 3 bytes, R-G-B order)
    """
    offset = blade * SK6805Driver.LEDS_PER_BLADE

    buf = driver._buf

    for i in range(SK6805Driver.LEDS_PER_BLADE):

        base = i * 3

        r = rgb_bytes[base]
        g = rgb_bytes[base + 1]
        b = rgb_bytes[base + 2]

        buf[offset + i] = (g << 24) | (r << 16) | (b << 8)


# ---------------------------------------------------------------------------
# Self-test  (run on Pico W with a short LED strip connected to GPIO 0)
# ---------------------------------------------------------------------------

def self_test():
    """
    Quick hardware validation sequence.
    Cycles R → G → B → white → off on all LEDs.
    Each state held for 500ms.
    Expected result: clean color transitions, no flicker, ~5ms show() latency.
    """
    import utime

    print("SK6805 Driver self-test starting...")
    driver = SK6805Driver(pin=0, num_leds=SK6805Driver.TOTAL_LEDS, brightness=64)

    colors = [
        ("RED",   255,   0,   0),
        ("GREEN",   0, 255,   0),
        ("BLUE",    0,   0, 255),
        ("WHITE", 255, 255, 255),
        ("OFF",     0,   0,   0),
    ]

    for name, r, g, b in colors:
        print(f"  → {name}")
        driver.fill(r, g, b)
        driver.show()
        utime.sleep_ms(500)

    # Blade isolation test: blade 0 = red, blade 1 = blue
    print("  → Blade isolation: blade0=RED, blade1=BLUE")
    for i in range(SK6805Driver.LEDS_PER_BLADE):
        driver.set_column(0, i, 255, 0, 0)
        driver.set_column(1, i, 0,   0, 255)
    driver.show()
    utime.sleep_ms(1000)

    driver.clear()
    driver.show()
    print("Self-test complete.")


# Run self-test when executed directly
if __name__ == "__main__":
    self_test()
