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
SLICES = 24
OUTPUT_FILE = "fan_image.bin"


# ==============================
# Argument Check
# ==============================

if len(sys.argv) < 2:
    print("Usage:")
    print("python3 LED_Conversion.py input_image.png")
    sys.exit(1)

input_image = sys.argv[1]


# ==============================
# Load Image
# ==============================

image = Image.open(input_image).convert("RGB")


size = min(image.size)

left = (image.width - size) // 2
top = (image.height - size) // 2
right = left + size
bottom = top + size

image = image.crop((left, top, right, bottom))
image = image.resize((size, size), Image.Resampling.LANCZOS)

width, height = image.size

center_x = width / 2
center_y = height / 2

pixels = image.load()


# ==============================
# Bilinear Sampling
# ==============================

def sample(x, y):

    x0 = int(x)
    y0 = int(y)

    x1 = min(x0 + 1, width - 1)
    y1 = min(y0 + 1, height - 1)

    dx = x - x0
    dy = y - y0

    r00, g00, b00 = pixels[x0, y0]
    r10, g10, b10 = pixels[x1, y0]
    r01, g01, b01 = pixels[x0, y1]
    r11, g11, b11 = pixels[x1, y1]

    r = (r00*(1-dx)*(1-dy) +
         r10*dx*(1-dy) +
         r01*(1-dx)*dy +
         r11*dx*dy)

    g = (g00*(1-dx)*(1-dy) +
         g10*dx*(1-dy) +
         g01*(1-dx)*dy +
         g11*dx*dy)

    b = (b00*(1-dx)*(1-dy) +
         b10*dx*(1-dy) +
         b01*(1-dx)*dy +
         b11*dx*dy)

    return int(r), int(g), int(b)


# ==============================
# Convert Cartesian → Polar
# ==============================

fan_data = bytearray()

for angle_index in range(SLICES):

    theta = 2 * math.pi * angle_index / SLICES

    slice_pixels = []

    for r in range(TOTAL_LEDS):

        radius = r / (TOTAL_LEDS - 1)

        x = center_x + radius * center_x * math.cos(theta)
        y = center_y + radius * center_y * math.sin(theta)

        x = max(0, min(width - 1.001, x))
        y = max(0, min(height - 1.001, y))

        r_val, g_val, b_val = sample(x, y)

        slice_pixels.append((r_val, g_val, b_val))

    blade0 = slice_pixels[:LEDS_PER_BLADE]
    blade1 = slice_pixels[LEDS_PER_BLADE:]

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
print("Slices:", SLICES)
