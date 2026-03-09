from LED_driver import SK6805Driver, make_slice_from_rgb_bytes
import utime

# ==============================
# Configuration
# ==============================

LEDS_PER_BLADE = 80
NUM_BLADES = 2
TOTAL_LEDS = LEDS_PER_BLADE * NUM_BLADES

SLICES = 360
BYTES_PER_LED = 3
SLICE_SIZE = TOTAL_LEDS * BYTES_PER_LED

IMAGE_FILE = "fan_image.bin"

# simulated rotation speed
RPM = 900
ROTATIONS_PER_SEC = RPM / 60
SLICES_PER_SEC = ROTATIONS_PER_SEC * SLICES

SLICE_DELAY_US = int(1_000_000 / SLICES_PER_SEC)

# ==============================
# Initialize Driver
# ==============================

driver = SK6805Driver(pin=0, num_leds=TOTAL_LEDS, brightness=128)

# ==============================
# Load Image
# ==============================

with open(IMAGE_FILE, "rb") as f:
    image_data = f.read()

NUM_SLICES = len(image_data) // SLICE_SIZE

print("Loaded slices:", NUM_SLICES)

# ==============================
# Display Loop
# ==============================

def display_loop():

    current_slice = 0

    while True:

        start = utime.ticks_us()

        offset = current_slice * SLICE_SIZE
        slice_data = image_data[offset:offset + SLICE_SIZE]

        blade0 = slice_data[:LEDS_PER_BLADE * 3]
        blade1 = slice_data[LEDS_PER_BLADE * 3:]

        make_slice_from_rgb_bytes(driver, 0, blade0)
        make_slice_from_rgb_bytes(driver, 1, blade1)

        driver.show()

        current_slice += 1

        if current_slice >= NUM_SLICES:
            current_slice = 0

        elapsed = utime.ticks_diff(utime.ticks_us(), start)

        if elapsed < SLICE_DELAY_US:
            utime.sleep_us(SLICE_DELAY_US - elapsed)


# ==============================
# Run
# ==============================

display_loop()
