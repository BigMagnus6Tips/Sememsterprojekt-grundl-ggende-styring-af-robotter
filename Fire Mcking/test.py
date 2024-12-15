from machine import Pin, ADC
from time import sleep

inputPin1 = Pin(19, Pin.IN, pull=Pin.PULL_DOWN)
inputPin2 = Pin(26, Pin.IN, pull=Pin.PULL_DOWN)
#inputPinADC = ADC(Pin(26))
try:
    while True:
        #voltage = inputPinADC.read_u16() * 3.3 / 65535
        print("Pin 19: ", inputPin1.value())
        print("Pin 26:", inputPin2.value())
        sleep(0.1)
except KeyboardInterrupt:
    pass