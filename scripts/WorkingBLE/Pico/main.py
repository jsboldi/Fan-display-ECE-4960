# ==============================================================================
# main.py — Top-level entry point for POV holographic fan (Pico W)
# Target: Raspberry Pi Pico W, MicroPython
#
# Execution flow
# --------------
# 1. Allocate both image buffers up front (avoid GC pauses later).
# 2. Start BLE peripheral (NUS) — begins advertising.
# 3. Start PIO LED driver.
# 4. Start IR-synced display controller.
# 5. Enter tight main loop:
#      a. Call display_controller.step() on every iteration.
#      b. Check if BLE receiver has a complete new image.
#      c. If so, swap buffers (zero-copy pointer swap).
# ==============================================================================

import bluetooth
import utime
from config import IMAGE_SIZE
from led_driver        import LEDDriver
from ble_receiver      import BLEImageReceiver
from display_controller import DisplayController

# ------------------------------------------------------------------------------
# 1. Allocate buffers once — never allocate inside the display loop.
# ------------------------------------------------------------------------------
active_buffer     = bytearray(IMAGE_SIZE)   # shown on LEDs right now
background_buffer = bytearray(IMAGE_SIZE)   # filled by BLE in the background

print("[MAIN] Buffers allocated: {} bytes each".format(IMAGE_SIZE))

# ------------------------------------------------------------------------------
# 2. Initialise subsystems
# ------------------------------------------------------------------------------
ble      = bluetooth.BLE()
receiver = BLEImageReceiver(ble, active_buffer, background_buffer)

driver   = LEDDriver()          # PIO state machine starts immediately
driver.blank()                  # ensure strip is dark at boot

display  = DisplayController(driver, active_buffer)

print("[MAIN] System ready — waiting for IR sync and BLE connection")

# ------------------------------------------------------------------------------
# 3. Main loop
# ------------------------------------------------------------------------------
while True:
    # ── Display: must run every iteration without blocking ────────────────────
    display.step()

    # ── Deferred BLE work (ACK sending — must NOT happen inside IRQ) ─────────
    receiver.tick()

    # ── Buffer swap: only when a full image has been received ─────────────────
    if receiver.image_ready():
        # Stop showing the strip momentarily is acceptable here:
        # the swap is a few microseconds of pointer exchange.
        receiver.swap_buffers()

        # After swap: receiver.active_buffer is the new image.
        # Point the display controller at it.
        display.set_active_buffer(receiver.active_buffer)

    # No sleep — the display loop must be as tight as possible.
    # utime.sleep_ms() or any blocking call here would miss slice windows.
