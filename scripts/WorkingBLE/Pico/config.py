# ==============================================================================
# config.py — Shared configuration for POV holographic fan system
# Import on both Pico (MicroPython) and PC (Python 3).
# ==============================================================================

# --- Hardware geometry ---
LEDS_PER_BLADE  = 80
NUM_BLADES      = 2
TOTAL_LEDS      = LEDS_PER_BLADE * NUM_BLADES   # 160
SLICES          = 32

# --- Binary image format ---
# Layout: [slice0·blade0·80LEDs][slice0·blade1·80LEDs][slice1·blade0·80LEDs]...
# Each LED = 3 bytes in GRB order (NOT RGB).
BYTES_PER_LED   = 3
IMAGE_SIZE      = SLICES * LEDS_PER_BLADE * NUM_BLADES * BYTES_PER_LED  # 15360

# Bytes for one complete slice (both blades)
SLICE_STRIDE    = LEDS_PER_BLADE * NUM_BLADES * BYTES_PER_LED  # 480

# --- BLE / Nordic UART Service (NUS) UUIDs ---
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Pico → PC (notify)
NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # PC → Pico (write)

# --- BLE transfer protocol ---
# The PC sends raw binary in chunks.  A 4-byte magic header marks the START
# of every new image so the Pico can resync after packet loss or reconnects.
TRANSFER_MAGIC   = b'\xDE\xAD\xBE\xEF'

# Safe BLE 4.x floor is 20 bytes; modern hosts negotiate up to 244 bytes.
# Set to what your host OS / adapter supports.  240 is a common safe value.
BLE_CHUNK_SIZE   = 100   # bytes per ATT write (conservative for Pico BLE stack)

# --- PIO / LED driver ---
PIO_STATE_MACHINE = 0    # which PIO state machine to use
LED_PIN           = 0    # GPIO pin connected to LED strip data line

# --- IR sync ---
IR_PIN            = 15   # GPIO for Hall-effect / IR sensor (active-low, pull-up)

# --- Timing ---
# If no IR pulse arrives within this window, display loop idles (fan stopped).
IR_TIMEOUT_MS     = 500
