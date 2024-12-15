from machine import Pin, PWM
from time import sleep

servo = PWM(Pin(15))  # Replace with your GPIO pin
servo.freq(50)  # 50 Hz for servo

# Test stop
servo.duty_u16(4915)
sleep(10)

# Test full speed in one direction
servo.duty_u16(6881)
sleep(2)

# Test full speed in the opposite direction
servo.duty_u16(2949)
sleep(2)

# Stop the servo
servo.duty_u16(4915)
