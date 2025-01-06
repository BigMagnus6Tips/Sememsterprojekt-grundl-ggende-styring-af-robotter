from machine import Pin, ADC
from time import sleep

greenReadLED = Pin(19, Pin.IN, pull=Pin.PULL_DOWN)
redReadLED = Pin(28, Pin.IN, pull=Pin.PULL_DOWN)
greenResetPin = Pin(18, Pin.OUT)
redResetPin = Pin(17, Pin.OUT)

try:
    redResetPin.value(0)
    while True:
        if redReadLED.value() == 1:
            print("Red LED is on")
            redResetPin.value(1)
            redResetPin.value(0)
        sleep(0.3)
except KeyboardInterrupt:
    pass