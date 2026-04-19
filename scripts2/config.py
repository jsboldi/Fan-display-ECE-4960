# ==============================================================================
# config.py — shared configuration for POV holographic fan
# ==============================================================================

# --- Hardware geometry (fixed, never changes) ---
LEDS_PER_BLADE  = 80
NUM_BLADES      = 2
TOTAL_LEDS      = LEDS_PER_BLADE * NUM_BLADES     # 160
BYTES_PER_LED   = 3                               # GRB

# --- Slice count (variable per transfer) ---
# SLICES is the DEFAULT at boot.  The actual slice count per image is read
# dynamically from the BLE transfer header.
SLICES          = 32
MAX_SLICES      = 64

# --- Derived sizes ---
SLICE_STRIDE    = LEDS_PER_BLADE * NUM_BLADES * BYTES_PER_LED   # 480
IMAGE_SIZE      = SLICES     * SLICE_STRIDE                     # 15360 (default)
MAX_IMAGE_SIZE  = MAX_SLICES * SLICE_STRIDE                     # 30720 (buffer alloc)

# --- BLE / Nordic UART Service (NUS) UUIDs ---
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

# --- BLE transfer protocol (8-byte header) ---
TRANSFER_MAGIC   = b'\xDE\xAD\xBE\xEF'
BLE_CHUNK_SIZE   = 100

# --- IR sync ---
IR_PIN            = 14
IR_TIMEOUT_MS     = 500
