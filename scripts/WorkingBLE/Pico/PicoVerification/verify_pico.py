# ==============================================================================
# verify_pico.py — Run on Pico via Thonny REPL after a successful transfer
#
# HOW TO USE:
#   After debug_receiver.py prints "IMAGE COMPLETE" and "Saved to flash":
#     1. Press Ctrl+C in Thonny to stop the loop
#     2. In the REPL at the bottom, paste:
#          exec(open('verify_pico.py').read())
#
# It will try sources in this order:
#   a) r.buffer  — in-RAM buffer from the still-running debug_receiver instance
#   b) fan_image.bin on flash — saved automatically by debug_receiver.tick()
# ==============================================================================

import gc

SLICES         = 32
LEDS_PER_BLADE = 80
NUM_BLADES     = 2
IMAGE_SIZE     = SLICES * LEDS_PER_BLADE * NUM_BLADES * 3   # 15360
SLICE_STRIDE   = LEDS_PER_BLADE * NUM_BLADES * 3            # 480
BLADE_STRIDE   = LEDS_PER_BLADE * 3                         # 240

# --- Find buffer ---
buf = None
source = ""

try:
    buf = r.buffer
    source = "r.buffer (in RAM)"
except NameError:
    pass

if buf is None or len(buf) != IMAGE_SIZE:
    try:
        with open("fan_image.bin", "rb") as f:
            buf = bytearray(f.read())
        source = "fan_image.bin (flash)"
    except OSError:
        print("ERROR: No buffer found.")
        print("  Option 1: Run debug_receiver, Ctrl+C after complete, then re-run this.")
        print("  Option 2: Make sure fan_image.bin was saved to flash.")
        raise SystemExit

print("\n== Pico Buffer Verification ==")
print("Source       :", source)
print("Buffer length:", len(buf), "(expected {})".format(IMAGE_SIZE),
      "OK" if len(buf) == IMAGE_SIZE else "*** MISMATCH ***")

if len(buf) != IMAGE_SIZE:
    raise SystemExit

# --- Basic stats ---
all_zero = all(b == 0 for b in buf)
print("All zeros?   :", all_zero, "<-- BAD: transfer failed" if all_zero else "")

nonzero = sum(1 for b in buf if b > 0)
print("Non-zero     : {}/{} ({}%)".format(nonzero, IMAGE_SIZE, nonzero * 100 // IMAGE_SIZE))

checksum = 0
for b in buf:
    checksum += b
checksum &= 0xFFFF
print("Checksum     : 0x{:04X}  <-- must match PC verify_image.py output".format(checksum))

# --- Per-slice brightness (compare visually with PC output) ---
print("\nPer-slice brightness (match this with PC verify_image.py output):")
for s in range(SLICES):
    off   = s * SLICE_STRIDE
    total = 0
    for i in range(SLICE_STRIDE):
        total += buf[off + i]
    avg = total // SLICE_STRIDE
    bar = avg * 20 // 255
    print("  Slice {:02d}: {} avg={:3d}".format(s, '#' * bar + '.' * (20 - bar), avg))

# --- Spot check specific LEDs ---
print("\nSpot-check LEDs (stored as GRB):")
spots = [
    (0,  0,  0,  "Slice00 Blade0 LED00 (inner center)"),
    (0,  0,  79, "Slice00 Blade0 LED79 (outer edge)  "),
    (0,  1,  0,  "Slice00 Blade1 LED00 (inner center)"),
    (16, 0,  40, "Slice16 Blade0 LED40 (mid)          "),
]
for (sl, bl, led, label) in spots:
    off = sl * SLICE_STRIDE + bl * BLADE_STRIDE + led * 3
    g, rv, b = buf[off], buf[off+1], buf[off+2]
    print("  {} G={:3d} R={:3d} B={:3d}".format(label, g, rv, b))

gc.collect()
print("\nFree mem:", gc.mem_free(), "bytes")
print("== Done ==\n")
print("Now run verify_image.py on your PC and compare the checksum and brightness map.")
