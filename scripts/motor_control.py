from machine import Pin, PWM
import time

class MotorController:
    def __init__(self, ena_pin, in1_pin, in2_pin, enc_a_pin, enc_b_pin, counts_per_rev=700):
        self.ena = PWM(Pin(ena_pin))
        self.ena.freq(1000)
        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        
        self.enc_a = Pin(enc_a_pin, Pin.IN, Pin.PULL_UP)
        self.enc_b = Pin(enc_b_pin, Pin.IN, Pin.PULL_UP)
        
        self.count = 0
        self.last_a = self.enc_a.value()
        self.counts_per_rev = counts_per_rev
        
        self.enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, 
                       handler=self._encoder_isr)
        
        # PID variables
        self.target_rpm = 0
        self.last_count = 0
        self.last_time = time.ticks_ms()
        self.integral = 0
        self.last_error = 0
        self.current_rpm = 0
        self.current_pwm = 0
        
        # PID gains - TUNED for better control
        self.kp = 300
        self.ki = 30
        self.kd = 15
        self.min_pwm = 6000  # Minimum PWM to start motor
    
    def _encoder_isr(self, pin):
        a = self.enc_a.value()
        b = self.enc_b.value()
        if a != self.last_a:
            self.count += 1 if a == b else -1
        self.last_a = a
    
    def _set_pwm(self, speed):
        """Internal: Set raw PWM, clamped to max"""
        speed = max(-65535, min(65535, speed))
        self.current_pwm = speed
        
        if speed > 0:
            self.in1.value(1)
            self.in2.value(0)
            self.ena.duty_u16(abs(speed))
        elif speed < 0:
            self.in1.value(0)
            self.in2.value(1)
            self.ena.duty_u16(abs(speed))
        else:
            self.in1.value(0)
            self.in2.value(0)
            self.ena.duty_u16(0)
    
    def set_target_rpm(self, rpm):
        """Set target speed in RPM"""
        self.target_rpm = rpm
        self.integral = 0
        self.last_error = 0
        self.last_count = self.count
        self.last_time = time.ticks_ms()
    
    def measure_current_rpm(self):
        """Just measure RPM without PID control"""
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000.0
        
        if dt < 0.05:
            return self.current_rpm
        
        count_diff = self.count - self.last_count
        counts_per_sec = count_diff / dt
        self.current_rpm = (counts_per_sec / self.counts_per_rev) * 60
        
        self.last_count = self.count
        self.last_time = now
        
        return self.current_rpm
    
    def update_pid(self):
        """Call this regularly (e.g. every 50-100ms)"""
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self.last_time) / 1000.0
        
        if dt < 0.05:
            return self.current_rpm
        
        # Calculate current RPM
        count_diff = self.count - self.last_count
        counts_per_sec = count_diff / dt
        self.current_rpm = (counts_per_sec / self.counts_per_rev) * 60
        
        # If target is 0, stop immediately
        if self.target_rpm == 0:
            self._set_pwm(0)
            self.integral = 0
            self.last_count = self.count
            self.last_time = now
            return self.current_rpm
        
        # PID calculation
        error = self.target_rpm - self.current_rpm
        self.integral += error * dt
        self.integral = max(-8000, min(8000, self.integral))
        derivative = (error - self.last_error) / dt if self.last_error != 0 else 0
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Apply minimum PWM threshold for low speeds
        if self.target_rpm > 0 and 0 < output < self.min_pwm:
            output = self.min_pwm
        
        # Apply output
        self._set_pwm(int(output))
        
        # Update state
        self.last_count = self.count
        self.last_time = now
        self.last_error = error
        
        return self.current_rpm
    
    def stop(self):
        self.target_rpm = 0
        self._set_pwm(0)
        self.integral = 0
    
    def get_count(self):
        return self.count
    
    def get_rpm(self):
        return self.current_rpm
    
    def get_pwm(self):
        return self.current_pwm
    
    def reset_count(self):
        self.count = 0

# Create motor
motor = MotorController(0, 1, 2, 3, 4, counts_per_rev=700)

print("="*50)
print("Measuring maximum RPM at full power...")
print("="*50)

# Run at full PWM to measure max speed
motor._set_pwm(65535)
time.sleep(0.5)  # Let it stabilize

max_rpm = 0
for i in range(30):
    rpm = abs(motor.measure_current_rpm())
    max_rpm = max(max_rpm, rpm)
    print(f"RPM: {rpm:.1f}")
    time.sleep(0.1)

motor.stop()
time.sleep(1)

print(f"\nMaximum RPM measured: {max_rpm:.1f}")
print("="*50)

# Now test PID at specific target speeds
test_speeds = [20, 50, 100, 150]

for target_speed in test_speeds:
    print(f"\nTesting at {target_speed} RPM")
    print("-"*50)
    
    motor.reset_count()
    motor.set_target_rpm(target_speed)
    
    for i in range(80):
        rpm = motor.update_pid()
        pwm = motor.get_pwm()
        error = target_speed - abs(rpm)
        print(f"T:{target_speed:3.0f} | C:{abs(rpm):6.1f} | E:{error:6.1f} | PWM:{pwm:5d}")
        time.sleep(0.1)
    
    motor.stop()
    time.sleep(2)

print("\n" + "="*50)
print("Test complete!")
print("="*50)
