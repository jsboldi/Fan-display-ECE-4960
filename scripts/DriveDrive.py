from machine import Pin, PWM, Timer
import machine
import time

# -------------------
# PIN CONFIGURATION
# -------------------
DIR_PIN = 0
PWM_PIN = 1
ENC_A_PIN = 2
ENC_B_PIN = 3

# -------------------
# MOTOR SETUP
# -------------------
dir_pin = Pin(DIR_PIN, Pin.OUT)
pwm = PWM(Pin(PWM_PIN))
pwm.freq(20000)

# -------------------
# ENCODER SETUP
# -------------------
enc_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)
encoder_count = 0

def encoder_handler(pin):
    global encoder_count
    if enc_b.value():
        encoder_count += 1
    else:
        encoder_count -= 1

enc_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_handler)

# -------------------
# PID PARAMETERS
# -------------------
Kp = 0.3
Ki = 0.05
Kd = 0.01
setpoint_rpm = 100
integral = 0
previous_error = 0
previous_count = 0
previous_time = time.ticks_ms()
COUNTS_PER_REV = 700
last_rpm = 0
last_output = 0
control_timer = Timer()
debug_timer = Timer()

# -------------------
# MOTOR CONTROL
# -------------------
def set_motor(speed):
    if speed >= 0:
        dir_pin.value(1)
    else:
        dir_pin.value(0)
        speed = -speed
    if speed > 100:
        speed = 100
    duty = int((speed / 100) * 65535)
    pwm.duty_u16(duty)

# -------------------
# MOTOR STOP
# -------------------
def motor_stop():
    global integral
    pwm.duty_u16(0)
    integral = 0
    control_timer.deinit()
    print("MOTOR STOPPED")

# -------------------
# PID LOOP (50Hz)
# -------------------
def control_loop(timer):
    global previous_count, previous_time
    global integral, previous_error
    global encoder_count
    global last_rpm, last_output

    irq_state = machine.disable_irq()
    current_count = encoder_count
    machine.enable_irq(irq_state)

    now = time.ticks_ms()
    dt = time.ticks_diff(now, previous_time) / 1000
    if dt <= 0:
        return

    delta = current_count - previous_count
    rpm = (delta / COUNTS_PER_REV) / dt * 60
    error = setpoint_rpm - rpm
    integral += error * dt
    derivative = (error - previous_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    output = max(-100, min(100, output))

    set_motor(output)
    previous_error = error
    previous_count = current_count
    previous_time = now
    last_rpm = rpm
    last_output = output

# -------------------
# DEBUG LOOP
# -------------------
def debug_loop(timer):
    error = setpoint_rpm - last_rpm
    print("Target RPM:", setpoint_rpm,
          "| Measured RPM:", round(last_rpm, 2),
          "| Count:", encoder_count,
          "| Output %:", round(last_output, 2),
          "| Error:", round(error, 2))

# -------------------
# START
# -------------------
control_timer.init(freq=50, mode=Timer.PERIODIC, callback=control_loop)
debug_timer.init(freq=2, mode=Timer.PERIODIC, callback=debug_loop)

# motor_stop()  # Uncomment to stop
