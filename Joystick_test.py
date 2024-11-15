from machine import Pin, ADC
import time

xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))

while True:
    time.sleep(0.1)
    xAxisValue = xAxis.read_u16()
    yAxisValue = yAxis.read_u16()
    # So in order to have 0,0 in the middle of the joystick, we need to subtract 32768 from the value
    # And in order to have a range from -1 to 1, we need to divide by 32768
    xAxisNorm = (xAxisValue - 32768) / 32768
    yAxisNorm = (yAxisValue - 32768) / 32768
    print("x: {:.2f} y: {:.2f}".format(xAxisNorm, yAxisNorm))
