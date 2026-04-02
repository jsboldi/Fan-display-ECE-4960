from machine import Pin, PWM
import time

# ====== CONFIG ======
ENC_A_PIN = 2
ENC_B_PIN = 3
PWM_PIN   = 1
DIR_PIN   = 0

CPR = 100
TARGET_RPM = 900
SAMPLE_TIME = 0.1  # PID loop speed

# ====== PID GAINS ======
Kp = 0.8
Ki = 0.5
Kd = 0.01

# ====== GLOBALS ======
encoder_count = 0

# ====== SETUP ======
enc_a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
enc_b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)

dir_pin = Pin(DIR_PIN, Pin.OUT)
dir_pin.value(1)

pwm = PWM(Pin(PWM_PIN))
pwm.freq(20000)

# ====== ISR ======
def encoder_isr(pin):
    global encoder_count
    
    if enc_b.value() == 1:
        encoder_count += 1
    else:
        encoder_count -= 1

enc_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr)

# ====== MOTOR STOP FUNCTION ======
def stop_motor():
    global integral
    pwm.duty_u16(0)
    integral = 0
    print("\nMotor stopped.\n")

# ====== PID VARIABLES ======
last_count = 0
last_time = time.ticks_ms()

integral = 0
last_error = 0

# For 1-second printing
print_timer = 0

print("Starting PID control to 900 RPM...\n")

try:
    while True:
        time.sleep(SAMPLE_TIME)
        
        # ---- TIME ----
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, last_time) / 1000
        
        # ---- RPM ----
        current_count = encoder_count
        delta_ticks = current_count - last_count
        rpm = (delta_ticks / CPR) * (60 / dt)
        
        # ---- ERROR ----
        error = TARGET_RPM - rpm
        
        # ---- PID ----
        integral += error * dt
        derivative = (error - last_error) / dt
        
        control = (Kp * error) + (Ki * integral) + (Kd * derivative)
        
        # ---- PWM ----
        duty = int(control * 50)
        
        if duty < 0:
            duty = 0
        if duty > 65535:
            duty = 65535
        
        pwm.duty_u16(duty)
        
        # ---- PRINT EVERY 1 SECOND ----
        print_timer += SAMPLE_TIME
        if print_timer >= 1.0:
            print("RPM:", round(rpm,1),
                  "Error:", round(error,1),
                  "PWM:", duty)
            print_timer = 0
        
        # ---- UPDATE ----
        last_count = current_count
        last_time = current_time
        last_error = error

except KeyboardInterrupt:
    stop_motor()
