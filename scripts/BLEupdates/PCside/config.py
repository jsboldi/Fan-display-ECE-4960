# config.py
# Shared constants for holographic fan display
# Both laptop and Pico W files import from this
# !! Lock these values before writing any other module !!

# ── LED / Blade Spec ───────────────────────────────────────
NUM_LEDS        = 80        # LEDs per blade
NUM_SLICES      = 90        # angular slices per rotation (2° resolution)
BYTES_PER_LED   = 3         # R, G, B
IMAGE_SIZE      = NUM_SLICES * NUM_LEDS * BYTES_PER_LED   # 21,600 bytes

# ── BLE ────────────────────────────────────────────────────
BLE_SERVICE_UUID        = "12345678-1234-1234-1234-123456789abc"
BLE_IMAGE_CHAR_UUID     = "12345678-1234-1234-1234-123456789abd"
BLE_CMD_CHAR_UUID       = "12345678-1234-1234-1234-123456789abe"
BLE_CHUNK_SIZE          = 244       # max BLE MTU payload in bytes
BLE_DEVICE_NAME         = "FanDisplay"

# ── BLE Commands ───────────────────────────────────────────
CMD_LED_ON              = 0x01
CMD_LED_OFF             = 0x02
CMD_SET_COLOR           = 0x03      # followed by R, G, B bytes
CMD_IMAGE_START         = 0x04      # begins image transfer
CMD_IMAGE_END           = 0x05      # ends image transfer
CMD_SET_BRIGHTNESS      = 0x06      # followed by 1 byte (0-255)

# ── Display ────────────────────────────────────────────────
LED_PIN         = 0         # GPIO pin, confirm from PCB schematic
BRIGHTNESS      = 0.3       # global brightness cap (0.0 - 1.0)
TARGET_RPM      = 900

# ── Flash Storage ──────────────────────────────────────────
IMAGE_FILE      = "image.bin"
