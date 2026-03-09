import math
from PIL import Image

LEDS_PER_BLADE = 80
NUM_BLADES = 2
TOTAL_LEDS = LEDS_PER_BLADE * NUM_BLADES

SLICES = 24
SLICE_SIZE = TOTAL_LEDS * 3

INPUT_FILE = "fan_image.bin"
OUTPUT_IMAGE = "preview.png"

PREVIEW_SIZE = 800

with open(INPUT_FILE, "rb") as f:
    data = f.read()

img = Image.new("RGB", (PREVIEW_SIZE, PREVIEW_SIZE), (0,0,0))
pixels = img.load()

center = PREVIEW_SIZE // 2
radius_max = PREVIEW_SIZE // 2

for slice_index in range(SLICES):

    theta1 = 2 * math.pi * slice_index / SLICES
    theta2 = 2 * math.pi * (slice_index + 1) / SLICES

    slice_offset = slice_index * SLICE_SIZE

    for r in range(TOTAL_LEDS):

        led_offset = slice_offset + r*3

        r_val = data[led_offset]
        g_val = data[led_offset+1]
        b_val = data[led_offset+2]

        radius = r / TOTAL_LEDS

        # fill between slices
        steps = 10

        for i in range(steps):

            theta = theta1 + (theta2 - theta1) * (i / steps)

            x = int(center + radius * radius_max * math.cos(theta))
            y = int(center + radius * radius_max * math.sin(theta))

            if 0 <= x < PREVIEW_SIZE and 0 <= y < PREVIEW_SIZE:
                pixels[x, y] = (r_val, g_val, b_val)

img.save(OUTPUT_IMAGE)

print("Preview saved:", OUTPUT_IMAGE)