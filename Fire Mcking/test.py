from machine import Pin, ADC
from time import sleep

greenReadLED = Pin(19, Pin.IN, pull=Pin.PULL_DOWN)
redReadLED = Pin(27, Pin.IN, pull=Pin.PULL_DOWN)
resetPin = Pin(18, Pin.OUT)
try:
    while True:
        if greenReadLED.value() == 1:
            print("Green LED is on")
            resetPin.value(1)
            resetPin.value(0)
        sleep(1)
except KeyboardInterrupt:
    pass