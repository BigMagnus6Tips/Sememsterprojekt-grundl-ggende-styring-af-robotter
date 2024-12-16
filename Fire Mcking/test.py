from machine import Pin, ADC
from time import sleep

greenReadLED = Pin(19, Pin.IN, pull=Pin.PULL_DOWN)
redReadLED = Pin(27, Pin.IN, pull=Pin.PULL_DOWN)
resetPin = Pin(18, Pin.OUT)
#inputPinADC = ADC(Pin(26))
try:
    while True:
        resetPin.toggle()
        sleep(1)
except KeyboardInterrupt:
    pass