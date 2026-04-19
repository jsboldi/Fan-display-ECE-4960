# ==============================================================================
# main.py — POV holographic fan top-level entry point
# Target: Raspberry Pi Pico W, MicroPython
#
# Boot sequence:
#   1. Allocate two MAX_IMAGE_SIZE buffers (fits up to MAX_SLICES = 64 slices)
#   2. Init BLE peripheral (starts advertising as "POV-Fan")
#   3. Init dual-blade PIO LED driver
#   4. Quick RGB self-test on both blades (visual wiring confirmation)
#   5. Init display controller (freerun mode, 32 slices default)
#   6. Enter main loop:
#        - display.step()      — output next slice if due
#        - receiver.tick()     — send ACK / re-advertise if needed
#        - swap buffers on full image receive
#        - 200 µs yield        — keeps BLE stack responsive
#
# FIX: sleep_ms(1) → sleep_us(200).
#
# With the led_driver interleaved-write fix, each write takes ~2,400 µs.
# The 16-slice window is 4,167 µs, leaving 1,667 µs of headroom.
# sleep_ms(1) consumed all of that headroom and caused ~8% timing slippage
# (slices fired 333 µs late every time, compressing the displayed image).
# sleep_us(200) reduces slippage to ~0.8% — barely perceptible.
# The BLE stack gets a yield every 200 µs which is more than sufficient;
# the CYW43 supervision timeout is measured in seconds.
# ==============================================================================

import bluetooth
import utime
from config             import MAX_IMAGE_SIZE, SLICE_STRIDE
from led_driver         import LEDDriver
from ble_receiver       import BLEImageReceiver
from display_controller import DisplayController

# ------------------------------------------------------------------------------
# 1. Allocate buffers (sized for max supported slice count; never reallocated)
# ------------------------------------------------------------------------------
active_buffer     = bytearray(MAX_IMAGE_SIZE)
background_buffer = bytearray(MAX_IMAGE_SIZE)
print("[MAIN] Buffers allocated: 2 x {} bytes".format(MAX_IMAGE_SIZE))

# ------------------------------------------------------------------------------
# 2. BLE peripheral
# ------------------------------------------------------------------------------
ble      = bluetooth.BLE()
receiver = BLEImageReceiver(ble, active_buffer, background_buffer)

# ------------------------------------------------------------------------------
# 3. LED driver
# ------------------------------------------------------------------------------
driver = LEDDriver()
driver.blank()

# ------------------------------------------------------------------------------
# 4. Boot self-test — flashes G, R, B on both blades (0.4 s each)
# ------------------------------------------------------------------------------
print("[MAIN] LED self-test...")
for _g, _r, _b in ((20, 0, 0), (0, 20, 0), (0, 0, 20)):
    _slice = bytearray([_g, _r, _b] * 160)
    driver.write(_slice)
    utime.sleep_ms(400)
driver.blank()
print("[MAIN] LED self-test complete")

# ------------------------------------------------------------------------------
# 5. Display controller (freerun until IR detected)
# ------------------------------------------------------------------------------
display = DisplayController(driver, active_buffer)
print("[MAIN] System ready — waiting for BLE image")

# ------------------------------------------------------------------------------
# 6. Main loop
# ------------------------------------------------------------------------------
while True:
    display.step()
    receiver.tick()

    if receiver.image_ready():
        receiver.swap_buffers()
        display.set_active_buffer(receiver.active_buffer,
                                  image_size=receiver.current_size)

    # FIX: was sleep_ms(1) — caused ~8% slice timing slippage.
    # 200 µs is enough to yield to the CYW43 BLE stack without meaningfully
    # eating into the 1,667 µs headroom inside each 16-slice window.
    utime.sleep_us(200)
