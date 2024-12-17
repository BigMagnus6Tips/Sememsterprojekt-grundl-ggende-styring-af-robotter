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
        # Checks if the lenght is the same as the steppermotors in the multistepper class
        if len(delays) != len(self.stepperMotors):
            print("must input a list of same size as stepperMotors")
            return 0
        
        # Sets the delays
        for i in range(len(self.stepperMotors)):
            self.stepperMotors[i].set_Delay(delays[i])
    
    # Function to set speed for motors as steps per second
    def set_Speed(self, speeds):
        if 0 in speeds:
            print("Speed must be non-zero")
            return 0       
        
        
        # Again chekcs the lenght of the speeds given
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


    # Function to stop all motors
    def stop(self):
        for motor in self.stepperMotors:
            motor.stop()
    
    # Function to get modes of the stepper motors
    def getModes(self):
        return [stepperMotor.mode for stepperMotor in self.stepperMotors]

    
    def getDelays(self):
        return [stepperMotor.delay for stepperMotor in self.stepperMotors]


# Class for the differential driver
class DifferentialDriver():
    fullStepsForFullRotation = 559
    
    def __init__(self, multiStepper):
        self.multiStepper = multiStepper
    # Steps to calculate steps for a given distance in 
    def distanceToSteps(self, distanceCm, mode):
        # The measuered diameter of the wheels
        diameter = 8.7
        # Calculates the circumference
        circumference = diameter*math.pi
        
        # Checks which mode the stepperMode is set to and calculates how many steps it needs
        stepsToGo = 0
        if mode == StepperMotor.full_step:
           stepsToGo = 200*distanceCm/circumference
        elif mode == StepperMotor.half_step:
            stepsToGo = 400*distanceCm/circumference
        elif mode == StepperMotor.micro_step:
           stepsToGo = 200*len(StepperMotor.micro_step_sequence)*distanceCm/(circumference*4)
            
        # Rounds the steps to go
        return round(stepsToGo)
    
    def stepsToDistance(self, steps, mode):
        diameter = 8.7
        circumference = diameter*math.pi
        distance = 0
        if mode == StepperMotor.full_step:
            distance = circumference*steps/200
        elif mode == StepperMotor.half_step:
            distance = circumference*steps/400
        elif mode == StepperMotor.micro_step:
            distance = circumference*steps/(200*len(StepperMotor.micro_step_sequence)/4)
        return distance
    
    # Function to go forward and backwards by making steps variable negative
    async def goForward(self, steps):
        # calls the move function for multistepper with steps
        await self.multiStepper.move([steps, steps])
    
    # Function to go forward a given distance
    async def goForwardGivenDistance(self, distanceCm):
        # Gets the steps for each motor taking into account the mode
        steps0 = self.distanceToSteps(distanceCm, self.multiStepper.getModes()[0])
        steps1 = self.distanceToSteps(distanceCm, self.multiStepper.getModes()[1])
        
        # Calls the move function for multistepper with steps
        await self.multiStepper.move([steps0, steps1])
    
    # Function to calculates steps to turn given amount of degrees
    def inPlaceRotationDegreesToSteps(self, degrees, mode):
 
        
        # checks which mode the stepperMode is set to and calculates how many steps it needs
        stepsToGo = 0
        if mode == StepperMotor.full_step:
           stepsToGo = self.fullStepsForFullRotation*degrees/360
        elif mode == StepperMotor.half_step:
            stepsToGo = 2*self.fullStepsForFullRotation*degrees/360
        elif mode == StepperMotor.micro_step:
           stepsToGo = self.fullStepsForFullRotation*len(StepperMotor.micro_step_sequence)/4
        
        # Rounds the steps to go
        return round(stepsToGo)
    
    def inPlaceRotationStepstoDegrees(self, steps, mode):
        degrees = 0
        if mode == StepperMotor.full_step:
            degrees = 360*steps/self.fullStepsForFullRotation
        elif mode == StepperMotor.half_step:
            degrees = 360*steps/(2*self.fullStepsForFullRotation)
        elif mode == StepperMotor.micro_step:
            degrees = 360*steps/(self.fullStepsForFullRotation*len(StepperMotor.micro_step_sequence)/4)
        
        return degrees
    
    # Function for rotating on place in both ways depending on if steps is negative og positiv
    async def inPlaceRotation(self, degree):
        print("inPlaceRotation")
        # Calculates the steps each motor has to take to turn given the degrees
        steps0 = self.inPlaceRotationDegreesToSteps(degree, self.multiStepper.getModes()[0])
        steps1 = self.inPlaceRotationDegreesToSteps(degree, self.multiStepper.getModes()[1])
        print(steps0, steps1)


        # Moves the different motors 2 different ways
        await self.multiStepper.move([steps0, -steps1])
    
    # Function to make a turn
    async def turn(self, steps, direction):
        # Checks if direction is positive or not and moves 1 of the motors depending on that
        if direction:
            await self.multiStepper.move([steps, 0])
        else:
            await self.multiStepper.move([0, steps])
        

class JoystickController:

    # We use a scaling factor of 400 because there are 400 steps in one revolution when using half steps
    def __init__(self, pin1, pin2, scalingFactor = 400):
        self.xAxis = ADC(Pin(pin1))
        self.yAxis = ADC(Pin(pin2))
        self.scalingFactor = scalingFactor
    
    def addButton(self, pin):
        self.button = Pin(pin, Pin.IN, Pin.PULL_UP)
    
    def readMovements(self):
        print(self.button.value())
        xAxisValue = self.xAxis.read_u16()
        yAxisValue = self.yAxis.read_u16()
        # So in order to have 0,0 in the middle of the joystick, we need to subtract 32768 from the value
        # And in order to have a range from -1 to 1, we need to divide by 32768
        xAxisNorm = -((xAxisValue - 32768) / 32768)
        yAxisNorm = -((yAxisValue - 32768) / 32768)
        
        # We multiply by the scaling factor to get a number that is more reasonable to work with
        xAxisNorm = round(xAxisNorm * self.scalingFactor)
        yAxisNorm = round(yAxisNorm * self.scalingFactor)

        deadZone = 0.1 * self.scalingFactor
        if yAxisNorm > deadZone:
            return [[-1, -1],[yAxisNorm, yAxisNorm],self.button.value()]
        elif yAxisNorm < -deadZone:
            return [[1, 1],[yAxisNorm, yAxisNorm],self.button.value()]
        elif xAxisNorm < -deadZone:
            return [[1, -1],[xAxisNorm, xAxisNorm],self.button.value()]
        elif xAxisNorm > deadZone:
            return [[-1, 1],[xAxisNorm, xAxisNorm],self.button.value()]
        else:
            return [[0, 0],[0, 0],self.button.value()]

class MonitorClass:
    def __init__(self, LDRPIN, REFERENCE_VOLTAGE, REFERENCE_RESISTANCE, LDROFFSET = [0, 0]):
        self.LDRPIN = ADC(Pin(LDRPIN))
        self.REFERENCE_VOLTAGE = REFERENCE_VOLTAGE
        self.REFERENCE_RESISTANCE = REFERENCE_RESISTANCE
        self.LDROFFSET = LDROFFSET
    
    def monitor(self):
            # Read the value from the LDR
            LDRValue = self.LDRPIN.read_u16()
            # Calculate the voltage
            voltage = LDRValue * self.REFERENCE_VOLTAGE / 65535
            return voltage
    
    def monitorDigital(self, VOLTAGE_CUTOFF):
        return self.monitor() < VOLTAGE_CUTOFF
    
    def getOffset(self):
        return self.LDROFFSET
    
    
class DeadReckoningHandler:
    def __init__(self, DifferentialDriver, position = [0, 0], angle = 0, boltDistance = 32.5):
        self.diffdriver = DifferentialDriver
        self.position = position
        self.angle = angle
        self.boltDistance = boltDistance



    async def rotate(self, angle):
        stepsToTake = self.diffdriver.inPlaceRotationDegreesToSteps(angle, self.diffdriver.multiStepper.getModes()[0])
        deltaAngle =  self.diffdriver.inPlaceRotationStepstoDegrees(stepsToTake, self.diffdriver.multiStepper.getModes()[0])
        self.angle += deltaAngle
        await self.diffdriver.inPlaceRotation(deltaAngle)

    async def goForwardInSteps(self, steps):
        self.position[0] += steps * math.cos(self.angle*math.pi/180)
        self.position[1] += steps * math.sin(self.angle*math.pi/180)
        await self.diffdriver.goForward(steps)


    def getPosition(self):
        return self.position
    
    def getPositionInCentimeter(self):
        return [self.diffdriver.stepsToDistance(self.position[0], self.diffdriver.multiStepper.getModes()[0]), self.diffdriver.stepsToDistance(self.position[1], self.diffdriver.multiStepper.getModes()[0])]

    def getAngle(self):
        return self.angle
    
    def setPosition(self, position):
        self.position = position
    
    def setAngle(self, angle):
        self.angle = angle

    async def moveToPoint(self, point):
        distance = [point[0] - self.diffdriver.stepsToDistance(self.position[0], self.diffdriver.multiStepper.getModes()[0]), point[1] - self.diffdriver.stepsToDistance(self.position[1], self.diffdriver.multiStepper.getModes()[0])]
        print("distance: " + str(distance))
        angle = math.atan2(distance[1], distance[0])*(360/(2 *math.pi)) - self.angle
        if angle > 180:
            angle -= 360
        if angle < -180:
            angle += 360

        await self.rotate(angle)
        await uasyncio.sleep(0.5)
        await self.goForwardInSteps(self.diffdriver.distanceToSteps(math.sqrt(distance[0]**2 + distance[1]**2),self.diffdriver.multiStepper.getModes()[0]))
    
    async def moveToPolar(self, distance, angle):
        await self.rotate(angle - self.angle)
        await uasyncio.sleep(0.5)
        await self.goForwardInSteps(self.diffdriver.distanceToSteps(distance,self.diffdriver.multiStepper.getModes()[0]))
    
    async def pickupAtPoint(self, point, crane = None):
        
        newPoint = [point[0] - self.boltDistance*math.cos(self.angle*math.pi/180), point[1] - self.boltDistance*math.sin(self.angle*math.pi/180)]
        await self.moveToPoint(newPoint)
        


    async def home(self, leftLDR, rightLDR, homePosition = [0, 0], homeAngle = 90, homingDistance = 223):
        self.leftCoordinates = []
        self.rightCoordinates = []
        
        testNumberOfDots = 100
        
        for _ in range(testNumberOfDots):
            ldrLine = await self.moveHoming(leftLDR, rightLDR, 2.2, homingDistance)
            if ldrLine == 0:
                    position = self.getPositionInCentimeter()
                    ldrPosition = leftLDR.getOffset()
                    self.leftCoordinates.append([position[0]+ldrPosition[0]*math.cos(self.getAngle()*math.pi/180)-position[1]+ldrPosition[1]*math.sin(self.getAngle()*math.pi/180),
                                                position[1]+ldrPosition[0]*math.sin(self.getAngle()*math.pi/180)+position[1]+ldrPosition[1]*math.cos(self.getAngle()*math.pi/180)])
                    await self.rotate(3)
        
            elif ldrLine == 1:
                    position = self.getPositionInCentimeter()
                    ldrPosition = rightLDR.getOffset()
                    self.rightCoordinates.append([position[0]+ldrPosition[0]*math.cos(self.getAngle()*math.pi/180)-position[1]+ldrPosition[1]*math.sin(self.getAngle()*math.pi/180),
                                                position[1]+ldrPosition[0]*math.sin(self.getAngle()*math.pi/180)+position[1]+ldrPosition[1]*math.cos(self.getAngle()*math.pi/180)])
                    await self.rotate(-3)
            elif ldrLine == 2:
                print("Homing done")
                break
        
        with open('left_coordinates.csv', 'w') as file:
            file.write('x,y\n')
            for coord in self.leftCoordinates:
                file.write(f'{coord[0]:.2f},{coord[1]:.2f}\n')

        with open('right_coordinates.csv', 'w') as file:
            file.write('x,y\n')
            for coord in self.rightCoordinates:
                file.write(f'{coord[0]:.2f},{coord[1]:.2f}\n')
        print(str(self.angle))
        self.setPosition(homePosition)
        self.setAngle(homeAngle)
        

    async def moveHoming(self, leftLDR, rightLDR, cutoff = 2.2, homingDistance = 223):
        rotateIndex = 1
        while True:

            leftValue = leftLDR.monitorDigital(cutoff)
            rightValue = rightLDR.monitorDigital(cutoff)
            if leftValue:
                return 0
            elif rightValue:
                return 1
            elif self.position[0]**2+self.position[1]**2 > (self.diffdriver.distanceToSteps(homingDistance, self.diffdriver.multiStepper.getModes()[0]))**2:
                print((self.position[0]**2+self.position[1]**2)**0.5)
                return 2 
            rotateIndex += 1
            if rotateIndex%100 == 0:
                await self.rotate(1)

            await self.goForwardInSteps(1)
            await uasyncio.sleep(0.0001)
                                

    async def turnHoming(self, leftLDR, rightLDR):
        pass 



class ServoMove:
    def __init__(self, pin, allowedRange, startAngle, frequency=50, ):
        """
        Initialize the servo motor with PWM control.
        :param pin: GPIO pin connected to the servo.
        :param frequency: PWM frequency (default 50 Hz for servos).
        """
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(frequency)
        self.angle = startAngle
        self.allowedRange = allowedRange
    def setAngle(self, angle):
        """
        Set the servo angle.
        :param angle: Angle in degrees (0 to 180).
        """
        if self.allowedRange[0] <= angle <= self.allowedRange[1]:
            duty = int(1000 + (angle / 180.0) * 8000)
            self.pwm.duty_u16(duty)
            self.angle = angle
        else:
            raise ValueError(f"Angle must be between {self.allowedRange[0]} and {self.allowedRange[1]}")

    async def servoMoveUp(self, start_angle, end_angle, step_size, delay):
        """
        Move the servo back and forth between start_angle and end_angle.
        :param start_angle: Starting angle (0 to 180).
        :param end_angle: Ending angle (0 to 180).
        :param step_size: Increment size for each movement.
        :param delay: Delay between each step (in seconds).
        :param task_delay: Pause before reversing the motion (in seconds).
        """
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")
        if start_angle >= end_angle:
            raise ValueError("Start angle must be less than end angle")

        try:
            # Move from start_angle to end_angle
            for current_angle in range(start_angle, end_angle, step_size):
                self.setAngle(current_angle)
                await uasyncio.sleep(delay)
        finally:
            self.deinit()

    async def servoMoveDown(self, start_angle, end_angle, step_size, delay):
    
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")
        if start_angle >= end_angle:
            raise ValueError("Start angle must be less than end angle")
        
        try:
            for current_angle in range(end_angle, start_angle, -step_size):
                self.setAngle(current_angle)
                await uasyncio.sleep(delay)
                print(f"Moving down: {current_angle}")
        
        finally:
            self.deinit()

    async def moveToAngle(self, angle, step_size, delay):
        """
        Move the servo to a specific angle.
        :param angle: Target angle (0 to 180).
        :param step_size: Increment size for each movement.
        :param delay: Delay between each step (in seconds).
        """
        print(f"moving to angle {angle}")
        if step_size <= 0:
            raise ValueError("Step size must be a positive integer")

        angleDiffrence = angle - self.angle
        # Move to the target angle
        if angleDiffrence <0:
            step_size = -step_size
        for current_angle in range(self.angle,angle, step_size):
            self.setAngle(current_angle)
            await uasyncio.sleep(delay)
        
    def deinit(self):
        """
        Deinitialize the PWM to release the GPIO pin.
        """
        self.pwm.deinit()

class Crane:
    def __init__(self, servo1, servo2, servo3, servo4, startAngles = [75, 30, 0, 0]):
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo3 = servo3
        self.servo4 = servo4
        self.startAngles = startAngles

    async def goToStart(self):
        await self.servo1.moveToAngle(self.startAngles[0], 2, 0.05)
        await self.servo2.moveToAngle(self.startAngles[1], 2, 0.05)
        await self.servo3.moveToAngle(self.startAngles[2], 2, 0.05)
        await self.servo4.moveToAngle(self.startAngles[3], 2, 0.05)

    async def toggleElectromagnet(self, pin_number, duration, frequency=1000, duty_cycle=50):
        
        electro = PWM(Pin(pin_number, Pin.OUT))
        electro.freq(frequency)
        electro.duty_u16(0)
        
        duty_u16 = int(duty_cycle / 100 * 65535)


        print("electro on")
        
        electro.duty_u16(duty_u16)

        await uasyncio.sleep(duration)

        electro.duty_u16(0)
        
        print("electro off")

    async def pickUpBolt(self):
        """
        Create and run servo tasks concurrently.
        """
        await uasyncio.gather(
            self.servo1.moveToAngle(100, 1, 0.05),
            self.servo2.moveToAngle(10, 1, 0.1),
            self.servo3.moveToAngle(128, 5, 0.05),
            self.servo4.moveToAngle(50, 1, 0.05)
        ) # type: ignore

        # Add a short delay before the next group of movements
        
        await uasyncio.gather(
            self.toggleElectromagnet(16, 0.5, 1000, 50),
            self.servo1.moveToAngle(110, 2, 0.05),
        ) # type: ignore
        
        await uasyncio.gather(
            self.toggleElectromagnet(16, 1, 1000, 50),
            self.servo3.moveToAngle(124, 2, 0.05),
            self.servo4.moveToAngle(35, 2, 0.05)
        ) # type: ignore
        
        await uasyncio.gather(
            self.toggleElectromagnet(16, 1, 1000, 50),
            self.servo3.moveToAngle(129, 2, 0.05),
            self.servo4.moveToAngle(70, 2, 0.05)
        ) # type: ignore
        
        await uasyncio.gather(
            self.toggleElectromagnet(16, 1, 1000, 50),
            self.servo1.moveToAngle(90, 2, 0.05),
            self.servo3.moveToAngle(137, 2, 0.05),
            self.servo4.moveToAngle(45, 2, 0.05)
        ) # type: ignore
        
        await uasyncio.gather(
            self.toggleElectromagnet(16, 0.5, 1000, 50),
            self.servo3.moveToAngle(120, 2, 0.05)
        ) # type: ignore
        await uasyncio.gather(
            self.toggleElectromagnet(16, 0.5, 1000, 50),
            self.servo1.moveToAngle(60, 5, 0.05)
        ) # type: ignore
        
        await uasyncio.gather(
            self.toggleElectromagnet(16, 1.5, 1000, 50),
            self.servo3.moveToAngle(10, 5, 0.05),
            self.servo4.moveToAngle(140, 5, 0.05)
        ) # type: ignore
        
        ## Move servos back to their positions concurrently
        await uasyncio.gather(
            self.servo4.moveToAngle(0, 5, 0.05),
            self.servo3.moveToAngle(0, 5, 0.05),
            self.servo2.moveToAngle(30, 5, 0.05),
            self.servo1.moveToAngle(75, 1, 0.05)
        ) # type: ignore
    
        # Keep the tasks running indefinitely
        #try:
        #    while True:
        #        await asyncio.sleep(1)
        #except asyncio.CancelledError:
        #    print("Shutting down...")

