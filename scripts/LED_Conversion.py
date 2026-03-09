import math
import struct
import sys
from PIL import Image

# ==============================
# Hardware Configuration
# ==============================

LEDS_PER_BLADE = 80
NUM_BLADES = 2
TOTAL_LEDS = LEDS_PER_BLADE * NUM_BLADES

SLICES = 360
BYTES_PER_LED = 3

OUTPUT_FILE = "fan_image.bin"


# ==============================
# Argument Check
# ==============================

if len(sys.argv) < 2:
    print("Usage:")
    print("python LED_Conversion.py input_image.png")
    sys.exit(1)

input_image = sys.argv[1]


# ==============================
# Load Image
# ==============================

image = Image.open(input_image).convert("RGB")

# Make image square
size = min(image.size)
image = image.crop((0, 0, size, size))
image = image.resize((size, size))

width, height = image.size

center_x = width / 2
center_y = height / 2

pixels = image.load()


# ==============================
# Convert Cartesian → Polar
# ==============================

fan_data = bytearray()

for angle_index in range(SLICES):

    theta = 2 * math.pi * angle_index / SLICES

    slice_pixels = []

    for r in range(TOTAL_LEDS):

        radius = r / TOTAL_LEDS

        x = center_x + radius * center_x * math.cos(theta)
        y = center_y + radius * center_y * math.sin(theta)

        xi = int(max(0, min(width - 1, x)))
        yi = int(max(0, min(height - 1, y)))

        r_val, g_val, b_val = pixels[xi, yi] 

        slice_pixels.append((r_val, g_val, b_val))

    # split into blade 0 and blade 1
    blade0 = slice_pixels[:LEDS_PER_BLADE]
    blade1 = slice_pixels[LEDS_PER_BLADE:]

    # encode in binary format
    for r_val, g_val, b_val in blade0:
        fan_data += struct.pack("BBB", r_val, g_val, b_val)

    for r_val, g_val, b_val in blade1:
        fan_data += struct.pack("BBB", r_val, g_val, b_val)


# ==============================
# Save Binary File
# ==============================

with open(OUTPUT_FILE, "wb") as f:
    f.write(fan_data)

print("Conversion complete!")
print("Input Image:", input_image)
print("Output File:", OUTPUT_FILE)
print("Total Size:", len(fan_data), "bytes")