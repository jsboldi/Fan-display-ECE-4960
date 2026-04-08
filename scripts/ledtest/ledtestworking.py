import array
import rp2
from machine import Pin
import utime

NUM_LEDS = 80  # LEDs per blade
BRIGHTNESS = 0.01  # 0 = off, 1 = full brightness

# ---------------- PIO program ----------------
@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24
)
def ws2812():
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [2]
    jmp(not_x, "do_zero")   .side(1)    [1]
    jmp("bitloop")          .side(1)    [4]
    label("do_zero")
    nop()                   .side(0)    [4]
    wrap()

# ---------------- Pins & State Machines ----------------
pin1 = Pin(15, Pin.OUT)  # Blade 1
sm1 = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=pin1)
sm1.active(1)

pin2 = Pin(20, Pin.OUT)  # Blade 2
sm2 = rp2.StateMachine(1, ws2812, freq=8_000_000, sideset_base=pin2)
sm2.active(1)

# ---------------- Buffers ----------------
buf1 = array.array("I", [0] * NUM_LEDS)
buf2 = array.array("I", [0] * NUM_LEDS)

# ---------------- Correct GRB packing ----------------
def pack_grb(r, g, b):
    """24-bit GRB value for SK6805 LEDs"""
    # Apply brightness scaling
    r = int(r * BRIGHTNESS)
    g = int(g * BRIGHTNESS)
    b = int(b * BRIGHTNESS)
    return (g << 16) | (r << 8) | b

def show():
    sm1.put(buf1, 8)
    sm2.put(buf2, 8)
    utime.sleep_us(100)

def clear_all():
    """Turn off all LEDs"""
    for i in range(NUM_LEDS):
        buf1[i] = 0
        buf2[i] = 0
    show()

# ---------------- Start test ----------------
print("Clearing all LEDs...")
clear_all()
utime.sleep(1)

print("Starting full color test on both blades at reduced brightness...")

while True:
    # RED
    clear_all()
    for i in range(NUM_LEDS):
        buf1[i] = pack_grb(255, 0, 0)
        buf2[i] = pack_grb(255, 0, 0)
    show()
    utime.sleep(1)

    # GREEN
    clear_all()
    for i in range(NUM_LEDS):
        buf1[i] = pack_grb(0, 255, 0)
        buf2[i] = pack_grb(0, 255, 0)
    show()
    utime.sleep(1)

    # BLUE
    clear_all()
    for i in range(NUM_LEDS):
        buf1[i] = pack_grb(0, 0, 255)
        buf2[i] = pack_grb(0, 0, 255)
    show()
    utime.sleep(1)

    # OFF
    clear_all()
    utime.sleep(1)
