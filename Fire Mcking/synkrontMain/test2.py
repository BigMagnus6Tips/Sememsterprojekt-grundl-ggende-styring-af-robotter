from RobotClasses import StepperMotor


motorRight = StepperMotor([0,1,2,3], 0.45, 18000)

motorRight.set_Delay(0.1)
motorRight.run()