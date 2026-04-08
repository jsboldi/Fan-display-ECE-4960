from machine import Pin, PWM
import time

# ====== CONFIG ======
ENC_A_PIN = 2
ENC_B_PIN = 3
PWM_PIN   = 1
DIR_PIN   = 0

CPR = 100
TARGET_RPM = 900
SAMPLE_TIME = 0.1

# ====== PID GAINS ======
Kp = 0.8
Ki = 0.6
Kd = 0.02

# ====== LIMITS ======
MAX_DUTY = 65535
MIN_DUTY = 0
INTEGRAL_LIMIT = 2000   # anti-windup clamp

# ====== GLOBALS ======
encoder_count = 0

# ====== SETUP ======
enc_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)

dir_pin = Pin(DIR_PIN, Pin.OUT)
dir_pin.value(1)

pwm = PWM(Pin(PWM_PIN))
pwm.freq(20000)
pwm.duty_u16(0)  # ensure OFF

# ====== ISR ======
def encoder_isr(pin):
    global encoder_count
    if enc_b.value() == 1:
        encoder_count += 1
    else:
        encoder_count -= 1

enc_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr)

# ====== MOTOR STOP ======
def stop_motor():
    global integral
    pwm.duty_u16(0)
    integral = 0
    print("\nMotor stopped.\n")

# ====== STARTUP DELAY ======
print("Power on. Waiting 5 seconds for safety...")
for i in range(3, 0, -1):
    print("Starting in", i)
    time.sleep(1)

print("\nStarting motor control...\n")

# ====== PID VARIABLES ======
last_count = 0
last_time = time.ticks_ms()

integral = 0
last_error = 0

# ====== FILTER ======
filtered_rpm = 0
alpha = 0.3   # smoothing factor (0–1)

# ====== PRINT TIMER ======
print_timer = 0

# ====== SOFT START ======
ramp_target = 0
RAMP_RATE = 30  # RPM per second

try:
    while True:
        time.sleep(SAMPLE_TIME)
        
        # ---- TIME ----
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, last_time) / 1000
        
        # ---- RPM ----
        current_count = encoder_count
        delta_ticks = current_count - last_count
        raw_rpm = (delta_ticks / CPR) * (60 / dt)
        
        # ---- FILTER (Exponential Moving Avg) ----
        filtered_rpm = (alpha * raw_rpm) + ((1 - alpha) * filtered_rpm)
        
        # ---- SOFT START RAMP ----
        if ramp_target < TARGET_RPM:
            ramp_target += RAMP_RATE * dt
            if ramp_target > TARGET_RPM:
                ramp_target = TARGET_RPM
        
        # ---- ERROR ----
        error = ramp_target - filtered_rpm
        
        # ---- PID ----
        integral += error * dt
        
        # Anti-windup clamp
        if integral > INTEGRAL_LIMIT:
            integral = INTEGRAL_LIMIT
        elif integral < -INTEGRAL_LIMIT:
            integral = -INTEGRAL_LIMIT
        
        derivative = (error - last_error) / dt
        
        control = (Kp * error) + (Ki * integral) + (Kd * derivative)
        
        # ---- PWM ----
        duty = int(control * 50)
        
        if duty < MIN_DUTY:
            duty = MIN_DUTY
        elif duty > MAX_DUTY:
            duty = MAX_DUTY
        
        pwm.duty_u16(duty)
        
        # ---- PRINT EVERY 1 SECOND ----
        print_timer += SAMPLE_TIME
        if print_timer >= 1.0:
            print("RPM:", round(filtered_rpm,1),
                  "Target:", round(ramp_target,1),
                  "PWM:", duty)
            print_timer = 0
        
        # ---- UPDATE ----
        last_count = current_count
        last_time = current_time
        last_error = error

except KeyboardInterrupt:
    stop_motor()
