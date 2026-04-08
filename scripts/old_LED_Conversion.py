# SEE Image Conversion folder

import math
import struct
import sys
from PIL import Image

# ==========================================
# Hardware Configuration
# ==========================================

LEDS_PER_BLADE = 80
NUM_BLADES = 2

SLICES = 32
OUTPUT_FILE = "fan_image.bin"


# ==========================================
# Argument Check
# ==========================================

if len(sys.argv) < 2:
    print("Usage:")
    print("python3 LED_Conversion.py input_image.png")
    sys.exit(1)

input_image = sys.argv[1]


# ==========================================
# Load Image
# ==========================================

image = Image.open(input_image).convert("RGB")

# crop to square
size = min(image.size)

left = (image.width - size) // 2
top = (image.height - size) // 2
right = left + size
bottom = top + size

image = image.crop((left, top, right, bottom))

# high quality resize
image = image.resize((size, size), Image.Resampling.LANCZOS)

width, height = image.size

center_x = width / 2
center_y = height / 2

pixels = image.load()


# ==========================================
# Bilinear Sampling
# ==========================================

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

    r = (
        r00 * (1-dx) * (1-dy) +
        r10 * dx * (1-dy) +
        r01 * (1-dx) * dy +
        r11 * dx * dy
    )

    g = (
        g00 * (1-dx) * (1-dy) +
        g10 * dx * (1-dy) +
        g01 * (1-dx) * dy +
        g11 * dx * dy
    )

    b = (
        b00 * (1-dx) * (1-dy) +
        b10 * dx * (1-dy) +
        b01 * (1-dx) * dy +
        b11 * dx * dy
    )

    return int(r), int(g), int(b)


# ==========================================
# Convert Cartesian → Polar
# ==========================================

fan_data = bytearray()

max_radius = min(center_x, center_y)

for slice_index in range(SLICES):

    theta = 2 * math.pi * slice_index / SLICES

    # ======================================
    # Blade 0
    # ======================================

    for led in range(LEDS_PER_BLADE):

        radius = (led + 0.5) / LEDS_PER_BLADE

        x = center_x + radius * max_radius * math.cos(theta)
        y = center_y + radius * max_radius * math.sin(theta)

        x = max(0, min(width - 1.001, x))
        y = max(0, min(height - 1.001, y))

        r_val, g_val, b_val = sample(x, y)

        fan_data += struct.pack("BBB", r_val, g_val, b_val)

    # ======================================
    # Blade 1 (180° opposite)
    # ======================================

    theta2 = theta + math.pi

    for led in range(LEDS_PER_BLADE):

        radius = (led + 0.5) / LEDS_PER_BLADE

        x = center_x + radius * max_radius * math.cos(theta2)
        y = center_y + radius * max_radius * math.sin(theta2)

        x = max(0, min(width - 1.001, x))
        y = max(0, min(height - 1.001, y))

        r_val, g_val, b_val = sample(x, y)

        fan_data += struct.pack("BBB", r_val, g_val, b_val)


# ==========================================
# Save Binary File
# ==========================================

with open(OUTPUT_FILE, "wb") as f:
    f.write(fan_data)


# ==========================================
# Output Stats
# ==========================================

print("Conversion complete!")
print("Input Image:", input_image)
print("Output File:", OUTPUT_FILE)
print("Slices:", SLICES)
print("Total Size:", len(fan_data), "bytes")

expected = SLICES * LEDS_PER_BLADE * NUM_BLADES * 3
print("Expected Size:", expected, "bytes")
