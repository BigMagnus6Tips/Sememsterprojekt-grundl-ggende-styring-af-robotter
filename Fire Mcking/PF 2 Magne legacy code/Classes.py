from machine import Pin, PWM
from time import sleep

class StepperMotor:
    step_counter = 0 
    pwm_max = 65535
    pwm_pct = 0
    pwm_val = int(pwm_max * pwm_pct)
    endloop = False



    def set_frequency(self, frequency):
        """
        Set the frequency for the PWM signals.

        :param frequency: Frequency for the PWM signals.
        """
        for pin in self.pins:
            pin.freq(frequency)


    def __init__(self, pins, pct = 0.5, frequency=18_000, stepsize=0, diameter=0):
        """
        initialize the stepper motor.

        :param pins: List of pins for the stepper motor.
        :param pct: Duty cycle percentage.
        :param frequency: Frequency for the PWM signals.
        :param stepsize: Step size for the stepper motor. (0: Full step, 1: Half step, 2: Micro step)
        :param diameter: Diameter of the wheel. (mm)
        
        """
        self.pins = [PWM(Pin(pin)) for pin in pins]
        self.set_frequency(frequency)
        self.pwm_pct = pct
        pwm_val = int(self.pwm_max * self.pwm_pct)
        # print(pwm_val)

        # Step sequences for the stepper motor.
        self.full_step= [
            [pwm_val, pwm_val, 0, 0],
            [0, pwm_val, pwm_val, 0],
            [0, 0, pwm_val, pwm_val],
            [pwm_val, 0, 0, pwm_val],
        ]
        self.half_step= [
            [pwm_val, 0, 0, 0],
            [pwm_val, pwm_val, 0, 0],
            [0, pwm_val, 0, 0],
            [0, pwm_val, pwm_val, 0],
            [0, 0, pwm_val, 0],
            [0, 0, pwm_val, pwm_val],
            [0, 0, 0, pwm_val],
            [pwm_val, 0, 0, pwm_val],
        ]
        self.micro_step= [
            [pwm_val, 0, 0, 0],
            [pwm_val, pwm_val//2, 0, 0],
            [pwm_val, pwm_val, 0, 0],
            [pwm_val//2, pwm_val, 0, 0],
            [0, pwm_val, 0, 0],
            [0, pwm_val, pwm_val//2, 0],
            [0, pwm_val, pwm_val, 0],
            [0, pwm_val//2, pwm_val, 0],
            [0, 0, pwm_val, 0],
            [0, 0, pwm_val, pwm_val//2],
            [0, 0, pwm_val, pwm_val],
            [0, 0, pwm_val//2, pwm_val],
            [0, 0, 0, pwm_val],
            [pwm_val//2, 0, 0, pwm_val],
            [pwm_val, 0, 0, pwm_val],
            [pwm_val, 0, 0, pwm_val//2],
            ]
        
        self.stepsize= [self.full_step, self.half_step, self.micro_step] # Step sequences for the stepper motor organised for quick access.
        self.stepsize_index = stepsize # Step size index for the stepper motor.
        self.distance_per_step = 3.14159 * diameter / (50*len(self.stepsize[self.stepsize_index])) # Distance travelled per step.
    
    def move_continuous(self, delay=0.001, direction=1):
        """
        Move the stepper motor continuously.
        
        :param delay: Delay between steps.
        :param direction: Direction to move the stepper motor (1: Clockwise, -1: Counter-clockwise
        """
        if direction == 1:
            while self.endloop == False:
                self.step_counter += 1
                for step in range(len(self.stepsize[self.stepsize_index])):
                    self.set_step(self.stepsize[self.stepsize_index][step])
                    sleep(delay)
        elif direction == -1:
            while self.endloop == False:
                self.step_counter -= 1
                for step in range(len(self.stepsize[self.stepsize_index])-1, -1, -1):
                    self.set_step(self.stepsize[self.stepsize_index][step])
                    sleep(delay)

        self.stop()
        
        
    
    def move(self, steps, delay=0.001, mode=0):
        """
        Move the stepper motor a specified number of steps.

        :param steps: Number of steps to move. (If mode is 1, steps is distance in mm)
        :param delay: Delay between steps.
        :param mode: Mode to move the stepper motor (0: steps, 1: distance)
        """
        if mode == 1:
            steps = int(steps/self.distance_per_step) # Convert distance to steps.

        if steps > 0:
            for cnt in range(steps):
                self.step_counter += 1
                self.set_step(self.stepsize[self.stepsize_index][self.step_counter%len(self.stepsize[self.stepsize_index])])
                '''for step in range(len(self.stepsize[self.stepsize_index])):
                    self.set_step(self.stepsize[self.stepsize_index][step])'''
                sleep(delay)
        elif steps < 0:
            for cnt in range(abs(steps)):
                self.step_counter -= 1
                self.set_step(self.stepsize[self.stepsize_index][self.step_counter%len(self.stepsize[self.stepsize_index])])
                '''for step in range(len(self.stepsize[self.stepsize_index])-1, -1, -1):
                    self.set_step(self.stepsize[self.stepsize_index][step])'''
                sleep(delay)
        self.stop()



    def set_step(self, step):
        """
        Set the step sequence for the stepper motor.

        :param step: Step sequence.
        """
        for pin in range(len(self.pins)):
            self.pins[pin].duty_u16(step[pin])
    
    def get_counter(self):
        """
        Get the current step counter value.
        """
        return self.step_counter
    
    def stop(self):
        """
        Stop the stepper motor.
        """
        self.set_step([0, 0, 0, 0])

class DifferentialDrive:
    def __init__(self, left, right, steps_per_rotation=1108):
        """
        Initialize the navigation system with two stepper motors.

        :param left: Instance of StepperMotor class for the left motor.
        :param right: Instance of StepperMotor class for the right motor.
        :param steps_per_rotation: Number of steps per rotation for the stepper motor.

        """

        self.left = left
        self.right = right
        self.distance_per_step = (self.left.distance_per_step + self.right.distance_per_step) / 2 # Distance travelled per step. (Average of the two motors)
        self.degree_per_step = 360 / steps_per_rotation # Degree travelled per steps
        # Ensure that all motors are turned off at
        self.stop()
    
    def move(self, steps, delay=0.01, mode=0):
        """
        Move the robot a specified number of steps.

        :param steps: Number of steps to move.
        :param delay: Delay between
        :param mode: Mode to move the stepper motor (0: steps, 1: distance)
        """
        if mode == 1:
            steps = int(steps/self.distance_per_step) # Convert distance to steps.
        # Move the robot.
        if steps > 0:
            for _ in range(steps):
                self.right.step_counter += 1
                self.left.step_counter += 1
                self.right.set_step(self.right.stepsize[self.right.stepsize_index][self.right.step_counter%len(self.right.stepsize[self.right.stepsize_index])])
                self.left.set_step(self.left.stepsize[self.left.stepsize_index][self.left.step_counter%len(self.left.stepsize[self.left.stepsize_index])])
                '''for step in range(len(self.stepsize[self.stepsize_index])):
                    self.set_step(self.stepsize[self.stepsize_index][step])'''
                sleep(delay)
        elif steps < 0:
            for _ in range(abs(steps)):
                self.left.step_counter -= 1
                self.right.step_counter -= 1
                self.right.set_step(self.right.stepsize[self.right.stepsize_index][self.right.step_counter%len(self.right.stepsize[self.right.stepsize_index])])
                self.left.set_step(self.left.stepsize[self.left.stepsize_index][self.left.step_counter%len(self.left.stepsize[self.left.stepsize_index])])
                '''for step in range(len(self.stepsize[self.stepsize_index])-1, -1, -1):
                    self.set_step(self.stepsize[self.stepsize_index][step])'''
                sleep(delay)
        self.stop()

    def rotate(self, steps, delay=0.01, mode=0):
        """
        Rotate the robot a certain angle at a certain speed.

        :param steps: Number of steps to rotate.
        :param delay: Delay between steps.
        :param mode: Mode to rotate the robot (0: steps, 1: angle)  

        """
        if mode == 1:
            steps = int(steps/self.degree_per_step)

        if steps > 0:
            for cnt in range(steps):
                self.left.step_counter += 1
                self.right.step_counter -= 1
                self.right.set_step(self.right.stepsize[self.right.stepsize_index][self.right.step_counter%len(self.right.stepsize[self.right.stepsize_index])])
                self.left.set_step(self.left.stepsize[self.left.stepsize_index][self.left.step_counter%len(self.left.stepsize[self.left.stepsize_index])])
                '''for step in range(len(self.stepsize[self.stepsize_index])):
                    self.set_step(self.stepsize[self.stepsize_index][step])'''
                sleep(delay)
        elif steps < 0:
            for cnt in range(abs(steps)):
                self.right.step_counter += 1
                self.left.step_counter -= 1
                self.right.set_step(self.right.stepsize[self.right.stepsize_index][self.right.step_counter%len(self.right.stepsize[self.right.stepsize_index])])
                self.left.set_step(self.left.stepsize[self.left.stepsize_index][self.left.step_counter%len(self.left.stepsize[self.left.stepsize_index])])
                '''for step in range(len(self.stepsize[self.stepsize_index])-1, -1, -1):
                    self.set_step(self.stepsize[self.stepsize_index][step])'''
                sleep(delay)
        self.stop()
    
    def rotate_one_wheel(self, steps, delay=0.01, wheel=0):
        """
        Rotate one wheel a certain number of steps.

        :param steps: Number of steps to rotate.
        :param delay: Delay between steps.
        :param wheel: Wheel to rotate (0: Left, 1: Right)

        """
        if steps > 0:
            for cnt in range(steps):
                if wheel == 0:
                    self.left.step_counter += 1
                    self.left.set_step(self.left.stepsize[self.left.stepsize_index][self.left.step_counter%len(self.left.stepsize[self.left.stepsize_index])])
                elif wheel == 1:
                    self.right.step_counter += 1
                    self.right.set_step(self.right.stepsize[self.right.stepsize_index][self.right.step_counter%len(self.right.stepsize[self.right.stepsize_index])])
                sleep(delay)
        elif steps < 0:
            for cnt in range(abs(steps)):
                if wheel == 0:
                    self.left.step_counter -= 1
                    self.left.set_step(self.left.stepsize[self.left.stepsize_index][self.left.step_counter%len(self.left.stepsize[self.left.stepsize_index])])
                elif wheel == 1:
                    self.right.step_counter -= 1
                    self.right.set_step(self.right.stepsize[self.right.stepsize_index][self.right.step_counter%len(self.right.stepsize[self.right.stepsize_index])])
                sleep(delay)
        self.stop()


    def stop(self):
        self.left.stop()
        self.right.stop()



