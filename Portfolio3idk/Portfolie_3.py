from machine import Pin, Timer, PWM, ADC
from time import sleep
import uasyncio
import math
#test2
class StepperMotor:
    
    # Define the step seqeunce for a full step
    full_step_sequence = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1],
    ]
    
    # Defines the step sequence for a half step
    half_step_sequence =[
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
    ]
    # Defines the step sequence of 32 steps to micro step
    micro_step_sequence=[
    [1, 0, 0, 0],
    [1, 0.2, 0, 0],
    [0.91, 0.4, 0, 0],
    [0.83, 0.55, 0, 0],
    [0.71, 0.71, 0, 0],
    [0.55, 0.83, 0, 0],
    [0.4, 0.91, 0, 0],
    [0.2, 1, 0, 0],
    
    [0, 1, 0, 0],
    [0, 1, 0.2, 0],
    [0, 0.91, 0.4, 0],
    [0, 0.83, 0.55, 0],
    [0, 0.71, 0.71, 0],
    [0, 0.55, 0.83, 0],
    [0, 0.4, 0.91, 0],
    [0, 0.2, 1, 0],
    
    [0, 0, 1, 0],
    [0, 0, 1, 0.2],
    [0, 0, 0.91, 0.4],
    [0, 0, 0.83, 0.55],
    [0, 0, 0.71, 0.71],
    [0, 0, 0.55, 0.83],
    [0, 0, 0.4, 0.91],
    [0, 0, 0.2, 1],
        
    [0, 0, 0, 1],
    [0.2, 0, 0, 1],
    [0.4, 0, 0, 0.91],
    [0.55, 0, 0, 0.83],
    [0.71, 0, 0, 0.71],
    [0.83, 0, 0, 0.55],
    [0.91, 0, 0, 0.4],
    [1, 0, 0, 0.2],    
    ]
    
    # Makes ids for the different steps to be used
    full_step = 0
    half_step = 1
    micro_step = 2
    
    # Makes the initializing function
    def __init__(self, pins, pwm_pct = 0.3, frequency=18_000, mode=1):
        # Makes all the pins used for the stepper motor PWM and saves them to the object
        
        self.pins = [PWM(Pin(pin))for pin in pins]
        # self.pins = [Pin(pin, Pin.OUT)for pin in pins]
        
        # sets the frequency to the given frequency
        self.frequency = frequency
        
        # sets it to go forward as standard
        self.forward = True
        
        # sets the PWM to all the pins
        for i in range(len(self.pins)):
            self.pins[i].freq(self.frequency)
        
        # Sets the PWM info for the duty cycle later
        self.pwm_pct = pwm_pct
        pwm_max = 65535
        self.pwm_val = int(pwm_max * self.pwm_pct)
        print(self.pwm_val)
        
        # sets the step count to 0 to keep track of steps
        self.step_count = 0
        
        # sets the mode for the stepper motor to full, half or microstepping
        self.mode = mode
        
        # sets an delay of 0.01 for standard
        self.delay = 0.01
        

    # function to set delay
    def set_Delay(self, delay):
        if delay < 0.0001:
            self.delay = 0.0001
            print(f"! Delay limit ! (delay set to {self.delay} instead)")
        else:
            self.delay = delay

    
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
        #checks if it shold step forward or backwards
        if self.forward:
            direction = 1 
        else:
            direction = -1

        # increases the step count or decresses it depending on going forward
        self.step_count += direction
        
        # given the mode the Stepper motor is set sets step to the correct step in the sequence
        if self.mode == StepperMotor.full_step:
            step = StepperMotor.full_step_sequence[self.step_count%len(StepperMotor.full_step_sequence)]
        elif self.mode == StepperMotor.half_step:
            step = StepperMotor.half_step_sequence[self.step_count%len(StepperMotor.half_step_sequence)]
        elif self.mode == StepperMotor.micro_step:
            step = StepperMotor.micro_step_sequence[self.step_count%len(StepperMotor.micro_step_sequence)]
        
        # changes the PWM for all the pins for the stepper motor depending on the step
        for pin in range(len(self.pins)):
            self.pins[pin].duty_u16(round(step[pin]*self.pwm_val))
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

    # Function to set delays for my stepperMotors
    def set_Delays(self, delays):
        # checks if the lenght is the same as the steppermotors in the multistepper class
        if len(delays) != len(self.stepperMotors):
            print("must input a list of same size as stepperMotors")
            return 0
        
        # sets the delays
        for i in range(len(self.stepperMotors)):
            self.stepperMotors[i].set_Delay(delays[i])
    
    # Function to set speed for motors as steps per second
    def set_Speed(self, speeds):
        
        # again chekcs the lenght of the speeds given
        if len(speeds) != len(self.stepperMotors):
            print("must input a list of same size as stepperMotors")
            return 0
        
        # Sets the delay to 1/speed for the motors 
        for i in range(len(self.stepperMotors)):
            self.stepperMotors[i].set_Delay(1/speeds[i])

    # An async function to move all stepper motors 
    async def move(self, steps):
        # Checks the lenght of the given steps to be the same as the number of motors
        if len(steps) != len(self.stepperMotors):
            print("must input a list of same size as stepperMotors")
            return 0
        
        # Create a list of tasks for both motors
        tasks = []
        
        # Loops through all motors
        for i in range(len(self.stepperMotors)):
            # Gets how many steps to take
            step_offset = steps[i]
            # Add each motor movement to the task list
            tasks.append(self.stepperMotors[i].move_Stepper(step_offset))
        
        # Wait for both motors to finish moving by gathering tasks
        # Even though it says it has an error it does not when the file is moved to the pico where it also works
        # Not sure why it says it has an error though
        await uasyncio.gather(*tasks) # type: ignore


    # function to stop all motors
    def stop(self):
        for motor in self.stepperMotors:
            motor.stop()
    
    # function to get modes of the stepper motors
    def getModes(self):
        return [stepperMotor.mode for stepperMotor in self.stepperMotors]


# Class for the differential driver
class DifferentialDriver():
    def __init__(self, multiStepper):
        self.multiStepper = multiStepper
    
    # steps to calculate steps for a given distance in 
    def distanceToSteps(self, distanceCm, mode):
        # the measuered diameter of the wheels
        diameter = 8.7
        # calculates the circumference
        circumference = diameter*math.pi
        
        # checks which mode the stepperMode is set to and calculates how many steps it needs
        stepsToGo = 0
        if mode == StepperMotor.full_step:
           stepsToGo = 200*distanceCm/circumference
        elif mode == StepperMotor.half_step:
            stepsToGo = 400*distanceCm/circumference
        elif mode == StepperMotor.micro_step:
           stepsToGo = 200*len(StepperMotor.micro_step_sequence)*distanceCm/(circumference*4)
            
        # rounds the steps to go
        return round(stepsToGo)
    
    
    # function to go forward and backwards by making steps variable negative
    async def goForward(self, steps):
        # calls the move function for multistepper with steps
        await self.multiStepper.move([steps, steps])
    
    # function to go forward a given distance
    async def goForwardGivenDistance(self, distanceCm):
        # gets the steps for each motor taking into account the mode
        steps0 = self.distanceToSteps(distanceCm, self.multiStepper.getModes()[0])
        steps1 = self.distanceToSteps(distanceCm, self.multiStepper.getModes()[1])
        
        # calls the move function for multistepper with steps
        await self.multiStepper.move([steps0, steps1])
    
    # Function to calculates steps to turn given amount of degrees
    def inPlaceRotationStepsToDegree(self, degrees, mode):
        fullStepsForFullRotation = 554
        
        # checks which mode the stepperMode is set to and calculates how many steps it needs
        stepsToGo = 0
        if mode == StepperMotor.full_step:
           stepsToGo = fullStepsForFullRotation*degrees/360
        elif mode == StepperMotor.half_step:
            stepsToGo = 2*fullStepsForFullRotation*degrees/360
        elif mode == StepperMotor.micro_step:
           stepsToGo = fullStepsForFullRotation*len(StepperMotor.micro_step_sequence)/4
        
        # rounds the steps to go
        return round(stepsToGo)
    
    # function for rotating on place in both ways depending on if steps is negative og positiv
    async def inPlaceRotation(self, degree):
        print("inPlaceRotation")
        # Calculates the steps each motor has to take to turn given the degrees
        steps0 = self.inPlaceRotationStepsToDegree(degree, self.multiStepper.getModes()[0])
        steps1 = self.inPlaceRotationStepsToDegree(degree, self.multiStepper.getModes()[1])
        print(steps0, steps1)


        # Moves the different motors 2 different ways
        await self.multiStepper.move([steps0, -steps1])
    
    # function to make a turn
    async def turn(self, steps, direction):
        # chekcs if direction is positive or not and moves 1 of the motors depending on that
        if direction:
            await self.multiStepper.move([steps, 0])
        else:
            await self.multiStepper.move([0, steps])
        

async def monitorStart():
    global shouldMonitor
    with open("FinalTestWithoutShield1.csv", "w") as file: # w is for write, which overwrites the file if it already exists
        file.write("Time (s), Voltage (V), Resistance (Ohm)\n")
        current_time = 0

        while shouldMonitor:
            # Read raw ADC value (12-bit range: 0 to 4095)
            adc_value = adc_pin.read_u16()  # This returns 16-bit value (0-65535)
            
            # Scale the 16-bit reading to voltage
            voltage = (adc_value / 65535) * REFERENCE_VOLTAGE

            # Calculate the resistance of the LDR with the formula from the voltage divider circuit
            resistance = round(REFERENCE_VOLTAGE * R2 / voltage - R2)
            
            # Print the data to the file, where 2f means there are two decimal points
            file.write("{:.2f}, {:.2f}, {}\n".format(current_time, voltage, resistance))

            # Delay between readings, which enables the robot to drive in the mean time.
            await uasyncio.sleep(0.1)
            # This is for plotting the time in the csv file.
            current_time += 0.1

# function to start the program
async def start():
    print("Starts")
    global shouldMonitor # Because then it can be used in the monitorStart function
    shouldMonitor = True
    uasyncio.create_task(monitorStart())
    await car.inPlaceRotation(180)
    shouldMonitor = False # Then the program stops monitoring.

if __name__ == '__main__':

    # Set up the ADC pin (choose one of GP26, GP27, or GP28 for ADC on Pico W)
    adc_pin = ADC(Pin(26))  # GP26 is labeled as ADC0 on Pico found in the Kicad drawing

    # Reference voltage for the Pico W is typically 3.3V
    REFERENCE_VOLTAGE = 3.3
    R2 = 2200  # Known resistor value in the voltage divider circuit. We chose this resistor to get a linear graph for the LDR, so that we could get as big of a span as possible.



    print("Starts")

    # Makes objects for the motor
    motorRight = StepperMotor([0,1,2,3], 0.15, 18000, StepperMotor.half_step)
    motorLeft = StepperMotor([4,5,6,7], 0.15, 18000, StepperMotor.half_step)

    # makes multistepper object with the motors
    multiStepper = MultiStepper([motorLeft, motorRight])

    # set their delays
    multiStepper.set_Delays([0.01,0.01])

    # makes a differentialDriver object
    car = DifferentialDriver(multiStepper)

    sleep(1)
    try:
        uasyncio.run(start())
    except:
        multiStepper.stop()