from Classes import StepperMotor, DifferentialDrive

try:

    if __name__ == "__main__":
        # Create an instance of the StepperMotor class for the left motor.
        left_motor = StepperMotor([0, 1, 2, 3], 0.5, 18000, 1,86)
        # Create an instance of the StepperMotor class for the right motor.
        right_motor = StepperMotor([4, 5, 6, 7], 0.5, 18000, 1,86)
        # Create an instance of the DifferentialDrive class.
        drive = DifferentialDrive(left_motor, right_motor)
        # Move the robot forward 400 steps.
        drive.move(1000,0.008,1)
        # Rotate the robot 400 steps.
        drive.rotate(90,0.008,1)
        drive.rotate_one_wheel(0,0.01,0)
        # Move the robot backward 400 steps.
        drive.move(1000,0.008,1)
        # Rotate the robot 400 steps
        drive.rotate(90,0.008,1)
        drive.move(1000,0.008,1)
        # Rotate the robot 400 steps
        drive.rotate(90,0.008,1)
        drive.move(1000,0.008,1)
        # Rotate the robot 400 steps
        drive.rotate(90,0.008,1)
except:
    drive.stop()
