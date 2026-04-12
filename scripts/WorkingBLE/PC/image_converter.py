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

BRIGHTNESS = 0.15  # Overall brightness (0.0 = off, 1.0 = full)
GAMMA     = 2.2   # Gamma correction — fixes color accuracy at low brightness.
                  # Without gamma, orange looks yellow, purple looks blue, etc.
                  # 2.2 matches human eye perception of LED intensity.

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

# Crop to square
size = min(image.size)
left = (image.width - size) // 2
top = (image.height - size) // 2
right = left + size
bottom = top + size
image = image.crop((left, top, right, bottom))

# High quality resize
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

    r = r00 * (1-dx) * (1-dy) + r10 * dx * (1-dy) + r01 * (1-dx) * dy + r11 * dx * dy
    g = g00 * (1-dx) * (1-dy) + g10 * dx * (1-dy) + g01 * (1-dx) * dy + g11 * dx * dy
    b = b00 * (1-dx) * (1-dy) + b10 * dx * (1-dy)           + b01 * (1-dx) * dy + b11 * dx * dy

    return int(r), int(g), int(b)

# ==========================================
# Gamma correction
# ==========================================

def gamma_correct(value):
    """
    Apply brightness scaling and gamma correction to a single channel value.
    Maps input 0-255 → output 0-255 with perceptual linearity.
    Without this, low-brightness colors look shifted (orange → yellow, etc).
    """
    if value == 0:
        return 0
    v = (value / 255.0) * BRIGHTNESS
    return int((v ** (1.0 / GAMMA)) * 255)

# ==========================================
# Convert Cartesian → Polar
# ==========================================

fan_data = bytearray()
max_radius = min(center_x, center_y)

for slice_index in range(SLICES):
    theta = 2 * math.pi * slice_index / SLICES

    # Blade 0
    for led in range(LEDS_PER_BLADE):
        radius = (led + 0.5) / LEDS_PER_BLADE
        x = center_x + radius * max_radius * math.cos(theta)
        y = center_y + radius * max_radius * math.sin(theta)
        x = max(0, min(width - 1.001, x))
        y = max(0, min(height - 1.001, y))
        r_val, g_val, b_val = sample(x, y)

        # Apply brightness + gamma correction.
        # gamma_correct() maps: pixel → (pixel/255 * brightness)^(1/gamma) * 255
        # This preserves perceptual color ratios at low brightness levels.
        r_val = gamma_correct(r_val)
        g_val = gamma_correct(g_val)
        b_val = gamma_correct(b_val)

        fan_data += struct.pack("BBB", g_val, r_val, b_val)

    # Blade 1 (180° opposite)
    theta2 = theta + math.pi
    for led in range(LEDS_PER_BLADE):
        radius = (led + 0.5) / LEDS_PER_BLADE
        x = center_x + radius * max_radius * math.cos(theta2)
        y = center_y + radius * max_radius * math.sin(theta2)
        x = max(0, min(width - 1.001, x))
        y = max(0, min(height - 1.001, y))
        r_val, g_val, b_val = sample(x, y)

        r_val = gamma_correct(r_val)
        g_val = gamma_correct(g_val)
        b_val = gamma_correct(b_val)

        fan_data += struct.pack("BBB", g_val, r_val, b_val)

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
