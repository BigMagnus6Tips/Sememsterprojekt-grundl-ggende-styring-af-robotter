# We have chosen just to keep everything in the RobotClasses file, even though we don't use all of it for Fire Mcking.
# The file is just here to be able to import it in the main.py file.
from machine import Pin, Timer, PWM, ADC
import uasyncio
import math
from time import sleep

class StepperMotor:

    # Define the step seqeunce for a full step
    full_step_sequence = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1],
    ]


    # Makes the initializing function
    def __init__(self, pins, pwm_pct = 0.3, frequency=18_000):
        # Makes all the pins used for the stepper motor PWM and saves them to the object

        self.pins = [PWM(Pin(pin))for pin in pins]
        # self.pins = [Pin(pin, Pin.OUT)for pin in pins]

        # sets the frequency to the given frequency
        self.frequency = frequency


        # sets the PWM to all the pins
        for i in range(len(self.pins)):
            self.pins[i].freq(self.frequency)

        # Sets the PWM info for the duty cycle later
        self.pwm_pct = pwm_pct
        pwm_max = 65535
        self.pwm_val = int(pwm_max * self.pwm_pct)

        # sets the step count to 0 to keep track of steps
        self.step_count = 0

        # sets an delay of 0.01 for standard
        self.delay = 0.01


    # function to set delay
    def set_Delay(self, delay):
        if delay < 0.0001:
            self.delay = 0.0001
            print(f"! Delay limit ! (delay set to {self.delay} instead)")
        else:
            self.delay = delay
            #print(self.delay)


    # function to set the PWM percantage
    def set_PWM(self, pwm_pct):
        self.pwm_pct= pwm_pct

    # function to set the frequency for the PWM
    def set_Frequency(self, frequency):
        self.frequency = frequency
        for i in range(len(self.pins)):
            self.pins[i].freq(self.frequency)

    # function to do a step
    def step(self):

        # increases the step count or decresses it depending on going forward
        self.step_count += 1
        # given the mode the Stepper motor is set sets step to the correct step in the sequence
        step = StepperMotor.full_step_sequence[self.step_count%len(StepperMotor.full_step_sequence)]

        # changes the PWM for all the pins for the stepper motor depending on the step
        for pin in range(len(self.pins)):
            self.pins[pin].duty_u16(step[pin]*self.pwm_val)
            # self.pins[pin].value(step[pin])
        # print("took a steep")

    # stops the PWM 
    def stop(self):
        for pin in range(len(self.pins)):
            self.pins[pin].duty_u16(0)
            #self.pins[pin].value(0)

    # makes current run again if needed 
    def resume(self):
        # since step() increases the stepcount we either substrack or add 1 to make sure we dont take another step using resume()
        if self.forward:
            self.step_count -= 1
        else:
            self.step_count += 1
        self.step()

    # returns the stepcount
    def get_Step_Count(self):
        return self.step_count

    # function to move stepper x amount of steps, also made it a async to makes sure it does not block the code 
    # for further implentation of multiple stepper motors
    async def move_Stepper(self, steps):
        # checks the direction to go
        if steps >= 0:
            self.forward = True
        else:
            self.forward = False
            # makes the steps positive if negative to go thgough it in a for loop
            steps = -steps

        # takes as many steps as specified
        for _ in range(steps):
            self.step()
            await uasyncio.sleep(self.delay)

        # sets the PWM duty cycle to 0 to make sure current does not run after use
        self.stop()

    # function to just make the stpeper motor run
    def run(self, forward = True):
        self.forward = forward
        while True:
            self.step_count += 1
            self.step()
            sleep(self.delay)


# Class for multiple stepper motors
class MultiStepper():
    # Argument stepperMotors should be a list of the stepperMotor class
    def __init__(self, stepperMotors):
        self.stepperMotors = stepperMotors
        self.syncDelay = 0.01

    # Function to set delays for my stepperMotors
    def set_Delays(self, delays):
        # Sets the delays
        for i in range(len(self.stepperMotors)):
            self.stepperMotors[i].set_Delay(delays[i])

    def setSyncDelay(self, delay):
        self.syncDelay = delay

    # Function to set speed for motors as steps per second
    def set_Speed(self, speeds):
        # Sets the delay to 1/speed for the motors 
        for i in range(len(self.stepperMotors)):
            self.stepperMotors[i].set_Delay(1/speeds[i])

    def moveSync(self, steps):
        maxSteps = max(steps)
        for i in range(maxSteps):
            if i < steps[0]:
                self.stepperMotors[0].step()
            if i < steps[1]:
                self.stepperMotors[1].step()
            sleep(self.syncDelay)
        self.stop()

    # Function to stop all motors
    def stop(self):
        for motor in self.stepperMotors:
            motor.stop()