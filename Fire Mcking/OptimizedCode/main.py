from machine import Pin
from OptimizedStepperClasses import StepperMotor, MultiStepper

# Precomputed constants
referenceVoltage = const(3.3)
r2 = const(2200)
threshold = const(0.75)
maxSpeed = const(750)
maxSpeedDelay = const(0.00133)
hardTurnSpeed = const(600)
hardTurnDelay = const(0.0016666)
resolution = const(50)
timeToMaxSpeed = const(0.5)
leftHard, rightHard = const(1), const(6)
leftSoft, rightSoft = const(1), const(7)

# Hardware Initialization
motorRight = StepperMotor(reversed([0, 1, 2, 3]), 0.35, 18000)
motorLeft = StepperMotor([4, 5, 6, 7], 0.35, 18000)
multiStepper = MultiStepper([motorLeft, motorRight])
redLedInput = Pin(28, Pin.IN, Pin.PULL_DOWN)
greenLedInput = Pin(19, Pin.IN, Pin.PULL_DOWN)
greenResetPin = Pin(18, Pin.OUT)
redResetPin = Pin(17, Pin.OUT)

# Utility Functions
def resetPin(pin):
    """Resets a pin by toggling its value."""
    pin.value(1)
    pin.value(0)

def rampUp(maxSpeed, resolution, timeToMaxSpeed):
    """Smoothly ramps up the stepper motor speed."""
    delay = maxSpeedDelay  # Start with the max speed delay
    for i in range(1, resolution + 1):
        moveValue = i * maxSpeed * timeToMaxSpeed / (resolution ** 2)
        multiStepper.setSyncDelay(delay)
        multiStepper.moveSync([moveValue] * 2)
        delay = 1 / (i * maxSpeed / resolution)  # Update delay inline

# Main Function
def main():
    counter = False
    rampUp(maxSpeed, resolution, timeToMaxSpeed)

    resetPin(redResetPin)
    resetPin(greenResetPin)
    multiStepper.setSyncDelay(maxSpeedDelay)

    while True:
        greenLed = greenLedInput.value()
        redLed = redLedInput.value()

        if greenLed:
            multiStepper.setSyncDelay(hardTurnDelay)
            while not redLedInput.value():  # Wait until red LED is detected
                multiStepper.moveSync([leftHard, rightHard])
            rampUp(hardTurnSpeed, 5, 0.2)
            resetPin(greenResetPin)

            while not redLedInput.value():  # Wait until red LED is detected
                multiStepper.moveSync([1, 0])
            multiStepper.setSyncDelay(maxSpeedDelay)
        elif redLed:
            multiStepper.moveSync([leftSoft, rightSoft])
            resetPin(redResetPin)
        else:
            # Toggle counter to alternate the movement pattern
            multiStepper.moveSync([2, 0] if counter else [3, 3])
            counter = not counter

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        multiStepper.stop()
