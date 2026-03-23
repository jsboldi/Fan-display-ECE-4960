from hall_sensor import HallSensor
from LED_driver import SK6805Driver, make_slice_from_rgb_bytes
import utime
# ==============================
# Configuration
# ==============================

LEDS_PER_BLADE = 80
NUM_BLADES = 2
TOTAL_LEDS = LEDS_PER_BLADE * NUM_BLADES

SLICES = 32
BYTES_PER_LED = 3
SLICE_SIZE = TOTAL_LEDS * BYTES_PER_LED

IMAGE_FILE = "fan_image.bin"


# ==============================
# Initialize Driver
# ==============================

driver = SK6805Driver(pin=0, num_leds=TOTAL_LEDS, brightness=128)
hall = HallSensor(pin=15)

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

    while True:

        rotation_period = hall.wait_for_rotation()

        slice_time = rotation_period // SLICES

        for current_slice in range(NUM_SLICES):

            start = utime.ticks_us()

            offset = current_slice * SLICE_SIZE
            slice_data = image_data[offset:offset + SLICE_SIZE]

            blade0 = slice_data[:LEDS_PER_BLADE * 3]
            blade1 = slice_data[LEDS_PER_BLADE * 3:]

            make_slice_from_rgb_bytes(driver, 0, blade0)
            make_slice_from_rgb_bytes(driver, 1, blade1)

            driver.show()

            elapsed = utime.ticks_diff(utime.ticks_us(), start)

            if elapsed < slice_time:
                utime.sleep_us(slice_time - elapsed)


display_loop()
