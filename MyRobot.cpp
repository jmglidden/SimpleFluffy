#include "WPILib.h"
#include "./PPC603gnu/Definitions.h"
#include "DigitalInput.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 
class RobotDemo : public SimpleRobot
{
	
	RobotDrive myRobot; // robot drive system
	Joystick controlStick; 
	Joystick leftDriveStick;
	Joystick rightDriveStick;
	Timer timer;
	Timer autoTimer;
	AnalogTrigger distanceSensorTrigger;
	AnalogChannel distanceSensorChannel;
	Gyro driveGyro; 
	
	
	private:
		bool clampState;
		Compressor compressor;
		Jaguar motorBasedGrabber;
		DigitalInput limitSwitch;
		Jaguar launcher;
		Solenoid clampPiston1; 
		Solenoid clampPiston2;
		Solenoid liftPiston1; 
		Solenoid liftPiston2;
		Jaguar leftMotor;
		Jaguar rightMotor;
		Encoder catapultCounter;
		Encoder leftEncoder;
		Encoder rightEncoder;

		
		

public:
	
	RobotDemo(void):
		myRobot(leftMotor, rightMotor),
		compressor(PRESSURE_SWITCH, COMPRESSOR_RELAY),
		controlStick(CONTROL_JOYSTICK),
		motorBasedGrabber(GRABBER_SPEED_CONTROLLER),  // the ball picker upper
		launcher(CATAPULT_SPEED_CONTROLLER),
		limitSwitch(CATAPULT_DOWN_INPUT),
		clampPiston1(CLAMP_PISTON_1),
		clampPiston2(CLAMP_PISTON_2),
		liftPiston1(LIFT_PISTON_1),
		liftPiston2(LIFT_PISTON_2),
		leftMotor(LEFT_MOTOR_SPEED_CONTROLLER),
		leftDriveStick(LEFT_DRIVE_INPUT),
		rightDriveStick(RIGHT_DRIVE_INPUT),
		catapultCounter(CATAPULT_ENCODER_1, CATAPULT_ENCODER_2),
		leftEncoder(LEFT_MOTOR_ENCODER_1, LEFT_MOTOR_ENCODER_2),
		rightEncoder(RIGHT_MOTOR_ENCODER_1, RIGHT_MOTOR_ENCODER_2),
		rightMotor(RIGHT_MOTOR_SPEED_CONTROLLER),
		distanceSensorTrigger(BALL_ULTRASONIC_CHANNEL),
		autoTimer(),
		distanceSensorChannel(BALL_ULTRASONIC_CHANNEL),
		timer(),
		driveGyro(1) //GYRO MUST BE ON PORT 1 or 2!!!!
		
		
		
		
	{
		catapultCounter.Start();
		catapultCounter.Reset();
		leftEncoder.Start();
		leftEncoder.Reset();
		rightEncoder.Start();
		rightEncoder.Reset();
		driveGyro.Reset();
		
		distanceSensorTrigger.SetLimitsVoltage(.01, BALL_DISTANCE); // Distance in inches = (BALL_DISTANCE * 10) + 1, Anything Below .07 (8in) becomes unpredictable 
		myRobot.SetExpiration(WATCHDOG_BISCUIT);
	}
	void Autonomous(void)
	{
		myRobot.SetSafetyEnabled(false);
	
		//int number = 0;
		static int autoNumber = 0;
		autoTimer.Start();
		while (IsAutonomous())
		{
			// Dont forget to change || back to && !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!		
			if ((Absf(rightEncoder.GetDistance()) >= RIGHT_DISTANCE_TO_TARGET && Absf(leftEncoder.GetDistance()) >= LEFT_DISTANCE_TO_TARGET) || autoTimer.Get() >= AUTO_TIMER_TICKS)
			{
				if (autoNumber < 1)
				{
				autoTimer.Stop();
				myRobot.Drive(0.0, 0.0);
				autoNumber++;
				}
				//ThrowControl(true);
				Wait(MOTOR_UPDATE_TIME);
				
#if ROBOT_DEBUG & AUTONOMOUS_ENCODER_TEST
				if (number >= PRINT_DELAY)
				{
					number = 0;
					printf("Autonomous Timer: %f\n", autoTimer.Get());
					printf("Autonomous Encoder Rate: %f\n", rightEncoder.GetRate());

				}
				number++;
#endif
				
			}
			else
			{
				//MockPID(AUTO_DRIVE_RATE);	
				myRobot.Drive(-0.5, 0.000001);
				
#if ROBOT_DEBUG & AUTONOMOUS_ENCODER_TEST
				if (number >= PRINT_DELAY)
				{
					number = 0;
					
					printf("Autonomous Encoder Rate: %f\n", leftEncoder.GetRate());
					printf("Autonomous Encoder ABS Distance: %f\n", Absf(leftEncoder.GetDistance()));
					printf("Autonomous Motor OutPut: %f\n", leftMotor.Get());
					printf("\n");
				}
				
				number++;
#endif
			}
			//printf(".");
			Wait(MOTOR_UPDATE_TIME);
		}
	}
	void OperatorControl(void)
	{
		//myRobot.SetSafetyEnabled(false);
		
		myRobot.SetSafetyEnabled(true);
		compressor.Start();
		driveGyro.Reset();
		
		//heading = 0;
		//gyroKP = -.03;
		while (IsOperatorControl())
		{
			
			
			
			
			

			//myRobot.TankDrive(0 - stick.GetRawAxis(2), 0 - stick.GetRawAxis(5)); Tank Drive from Gamepad
			//MotorBasedGrabber(controlStick); El-Torro
			
			//myRobot.TankDrive(0 - (1 * leftDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)), 0 - (1 * rightDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)));
			
			
			//myRobot.ArcadeDrive( -(leftDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)), -(rightDriveStick.GetRawAxis(JOYSTICK_X_AXIS)));
			static int turnState = TURNING;
			//static float gyroKP = -.03;
			static float offset = 0;
			static float heading = 0;
			offset = (heading - driveGyro.GetAngle()) * -GYRO_KP;
			switch (turnState)
			{
			case NOT_TURNING:
				if (rightDriveStick.GetRawAxis(JOYSTICK_X_AXIS) >= .1 || rightDriveStick.GetRawAxis(JOYSTICK_X_AXIS) <= -.1)
				{
					turnState = TURNING;
					printf("Turning \n");
					
					
				}
				else 
				{
					//printf("Driving");
					myRobot.ArcadeDrive(0 - (1 * leftDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)), offset);
				
				}
				break;
			case TURNING:
				if (!(rightDriveStick.GetRawAxis(JOYSTICK_X_AXIS) >= .1 || rightDriveStick.GetRawAxis(JOYSTICK_X_AXIS) <= -.1))
				{
					turnState = NOT_TURNING;
					printf("Not Turning \n");
					heading = driveGyro.GetAngle();
					printf("Heading %f\n", heading);
				}
				else
				{
					myRobot.ArcadeDrive( -(leftDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)), -(rightDriveStick.GetRawAxis(JOYSTICK_X_AXIS)));
				}
				break;
			}
			
#if ROBOT_DEBUG & SENSOR_TEST
			static int number1 = 0;
			if (number1 >= 200)
			{
				number1 = 0;
			//printf("Distance In Range: %f\n", distanceSensorTrigger.GetInWindow());
			//printf("Ultrasonic Voltage: %f\n", distanceSensorChannel.GetVoltage());
			//printf("Ultrasonic Value: %f\n", distanceSensorChannel.GetValue());
			//printf("Ultrasonic Average Voltage: %f\n", distanceSensorChannel.GetAverageVoltage());
			//printf("Ultrasonic Average Value: %f\n", distanceSensorChannel.GetAverageValue());
			//printf(" %s\n", distanceSensorChannel.GetModule());
			printf("Drive Gyro Heading: %f\n", heading);
			printf("Gyro Offset: %f\n", offset);
			
			}
			number1++;
#endif
			
			
			
			
			
			
			/*	static int driveMode = TANK;
			static int driveDelay = INITIAL_DELAY;
			if (leftDriveStick.GetRawButton(LEFT_DRIVE_TOGGLE_1) && driveDelay <= INITIAL_DELAY && driveMode == TANK)
			{
				
				if (leftDriveStick.GetRawButton(LEFT_DRIVE_TOGGLE_2) && rightDriveStick.GetRawButton(RIGHT_DRIVE_TOGGLE_1) && rightDriveStick.GetRawButton(RIGHT_DRIVE_TOGGLE_2) && driveMode == TANK)
				{
					driveMode = ARCADE;
					driveDelay = 1;
				}
				else if (leftDriveStick.GetRawButton(LEFT_DRIVE_TOGGLE_1) && driveDelay <= INITIAL_DELAY && driveMode == ARCADE)
				{
					
					if (leftDriveStick.GetRawButton(LEFT_DRIVE_TOGGLE_2) && rightDriveStick.GetRawButton(RIGHT_DRIVE_TOGGLE_1) && rightDriveStick.GetRawButton(RIGHT_DRIVE_TOGGLE_2) && driveMode == ARCADE)
					{
				
					driveMode = TANK;
					driveDelay = 1;
					}
				}
				else if (!leftDriveStick.GetRawButton(LEFT_DRIVE_TOGGLE_1))
				{
					
					if (!leftDriveStick.GetRawButton(8) && !rightDriveStick.GetRawButton(RIGHT_DRIVE_TOGGLE_1) && !rightDriveStick.GetRawButton(RIGHT_DRIVE_TOGGLE_2))
					{
					driveDelay = INITIAL_DELAY;
					}
				}
			}
			switch(driveMode)
			{
			case(TANK):
				myRobot.TankDrive(0 - (1 * leftDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)), 0 - (1 * rightDriveStick.GetRawAxis(JOYSTICK_Y_AXIS)));
			printf("TankDrive \n");
			break;
			case(ARCADE):
				myRobot.ArcadeDrive(rightDriveStick); // drive with arcade style (use right stick)
			printf("ArcadeDrive \n");
			break;
			}
			*/
			PneumaticGrabber();
			PneumaticLifter();
			ThrowControl();
			//StepVariable();

#if ROBOT_DEBUG & ENCODER_TEST
			printf("Encoder Distance: %f\n", rightEncoder.GetDistance());
			printf("Encoder PID Get: %f\n", rightEncoder.PIDGet());
			printf("Encoder Rate: %f\n", rightEncoder.GetRate());
#endif
			Wait(MOTOR_UPDATE_TIME);	// wait for a motor update time
			
		}
	}

	void MockPID(double setPoint)
	{
		static double rightMotorStep = 0.001;
		static double leftMotorStep = 0.001;
		
		static float rightMotorOutput = 0;
		double rightMotorSpeed = rightEncoder.GetRate();
		
		
		if (rightMotorSpeed >= (-setPoint))
		{
			if ((Absf(setPoint) - Absf(rightEncoder.GetRate())) <= 500 && (Absf(setPoint) - Absf(rightEncoder.GetRate())) >= -500)
				{
					rightMotorStep = .0001;
				}
			else
				{
					rightMotorStep = .001;
				}
				
			rightMotorOutput = Limit(rightMotorOutput + rightMotorStep);
		}
		else
		{
			rightMotorOutput = Limit(rightMotorOutput - rightMotorStep);
		}
			rightMotor.Set(rightMotorOutput);
		
		static float leftMotorOutput = 0;
		double leftMotorSpeed = leftEncoder.GetRate();
		
		if (leftMotorSpeed <= (setPoint))
		{
			if ((Absf(setPoint) - Absf(leftEncoder.GetRate())) <= 500 && (Absf(setPoint) - Absf(leftEncoder.GetRate())) >= -500)
			{
				leftMotorStep = .0001;
			}
			else
			{
				leftMotorStep = .001;
			}
			leftMotorOutput = Limit(leftMotorOutput - leftMotorStep);
		}
		else
		{
			leftMotorOutput = Limit(leftMotorOutput + leftMotorStep);
		}
			leftMotor.Set(leftMotorOutput);

	}
	
	double Limit (double input)
	{
		if (input >= 1.0)
		{
			return 1.0;
		}
		if (input <= -1.0)
		{
			return -1.0;
		}
		return input;
	}
	
	// Absolute value for f*****
	double Absf (double number)
	{
		if (number < 0)
		{
			number *= -1;
		}
			return number;
		
	}
	
	void ThrowControl(bool override = false, int throws = 1)
	{
		static int throwState = THROW_STATE_DOWN;
		static int delay = INITIAL_DELAY;
		static int counter = 0;
		
		switch (throwState)
		{
			case THROW_STATE_DOWN:
				timer.Reset();
				
				if ((controlStick.GetRawButton(CATAPULT_THROW_BUTTON) && delay == INITIAL_DELAY) || (override && counter < throws))
				{
					
#if ROBOT_DEBUG & CATAPULT_STATE
					printf("Settling \n");
#endif
					
					throwState = THROW_STATE_SETTLING;
					delay++;
					counter++;
					compressor.Stop();
					if (clampState || override)
					{
						
#if ROBOT_DEBUG & CATAPULT_STATE
						printf("Opening \n");
#endif
						
					clampPiston1.Set(false);
					clampPiston2.Set(true);
					Wait(1);
					}
					
				}
				break;
				
			case THROW_STATE_SETTLING:
							
				timer.Start();
				if (catapultCounter.GetDistance() >= CATAPULT_SETTLING_DISTANCE || timer.Get() >= CATAPULT_SETTLING_TIMER_TICKS)
				{
					
#if ROBOT_DEBUG & CATAPULT_STATE	
					printf("Encoder Distance: %f\n", catapultCounter.GetDistance());
					printf("Encoder Rate: %f\n", catapultCounter.GetRate());
					printf("Timer Value: %f\n", timer.Get());
					printf("Shooting \n");
#endif
					
					launcher.Set(MOTOR_OFF_SPEED);
						throwState = THROW_STATE_SHOOTING;
						timer.Reset();
				}
				else 
				{
					launcher.Set(CATAPULT_SETTLING_SPEED);
				}
				break;
				
			case THROW_STATE_SHOOTING:
				
				timer.Start();
				if (catapultCounter.GetDistance() >= CATAPULT_DISTANCE || timer.Get() >= CATAPULT_TIMER_TICKS)
				{
					
#if ROBOT_DEBUG & CATAPULT_STATE
					printf("Encoder Distance: %f\n", catapultCounter.GetDistance());
					printf("Encoder Rate: %f\n", catapultCounter.GetRate());
					printf("Timer Value: %f\n", timer.Get());
					printf("Resetting \n");
#endif
					
					launcher.Set(MOTOR_OFF_SPEED);
						throwState = THROW_STATE_RESETTING;
						timer.Reset();
						compressor.Start();
				}
				else 
				{
					launcher.Set(CATAPULT_FORWARD_SPEED);
				}
				
				/*if (controlStick.GetRawButton(CATAPULT_RESET_BUTTON))
				{
					
#if ROBOT_DEBUG & CATAPULT_STATE
					printf("Resetting \n");
#endif
					
					launcher.Set(MOTOR_OFF_SPEED);
					throwState = THROW_STATE_RESETTING;
				}
				*/
				break;
			case THROW_STATE_RESETTING:
				
				timer.Start();
				if (catapultCounter.GetDistance() <= CATAPULT_INITIAL_DISTANCE + 50  || timer.Get() >= CATAPULT_TIMER_DOWN_TICKS)
				{
					launcher.Set(MOTOR_OFF_SPEED);
					if (!controlStick.GetRawButton(CATAPULT_THROW_BUTTON))
					{
						
#if ROBOT_DEBUG & CATAPULT_STATE
						printf("Timer Value: %f\n", timer.Get());
						printf("Encoder Distance: %f\n", catapultCounter.GetDistance());
						printf("Reset \n");
#endif
						
						delay = INITIAL_DELAY;
						throwState = THROW_STATE_DOWN;
					}
					
				}
				else
				{
					launcher.Set(CATAPULT_REVERSE_SPEED);
				}
				break;
				
		}
			
		
		
	}
	
	void PneumaticGrabber()
	{
		static int grabState = CLOSED_STATE;
		static int delay = INITIAL_DELAY;
		if(controlStick.GetRawButton(CLAMP_BUTTON) && grabState == CLOSED_STATE && delay == INITIAL_DELAY /*&& distanceSensorTrigger.GetInWindow()*/)
				{
			
			//Opens Ball Clamp
			//True clampState means closed
#if ROBOT_DEBUG & GRABBER_STATE
					printf("Open Clamps \n");
#endif
					clampPiston1.Set(true);
					clampPiston2.Set(false);
					grabState = OPEN_STATE;
					delay++;
					clampState = true;
				}
				else if(controlStick.GetRawButton(CLAMP_BUTTON) && grabState == OPEN_STATE && delay == INITIAL_DELAY)
				{
					
			//Closes Ball Clamp
			//False clampState means Open
#if ROBOT_DEBUG & GRABBER_STATE
					printf("Close Clamps \n");
#endif
					clampPiston1.Set(false);
					clampPiston2.Set(true);
					grabState = CLOSED_STATE;
					delay++;
					clampState = false;
				}
				else if (!controlStick.GetRawButton(CLAMP_BUTTON))
				{
					delay = INITIAL_DELAY;
				}
	}
	
	void PneumaticLifter()
		{
			static int liftState = UP_STATE;
			static int delay = INITIAL_DELAY;
			if(controlStick.GetRawButton(LIFT_BUTTON) && liftState == DOWN_STATE && delay == INITIAL_DELAY)
					{
				
				//Lifts Clamp
#if ROBOT_DEBUG & LIFTER_STATE
						printf("Raise Grabber \n");
#endif
						liftPiston1.Set(true);
						liftPiston2.Set(false);
						liftState = UP_STATE;
						delay++;
						
					}
					else if(controlStick.GetRawButton(LIFT_BUTTON) && liftState == UP_STATE && delay == INITIAL_DELAY)
					{
						
				//Lowers Clamp
#if ROBOT_DEBUG & LIFTER_STATE						
						printf("Lower Grabber \n");
#endif
						liftPiston1.Set(false);
						liftPiston2.Set(true);
						liftState = DOWN_STATE;
						delay++;
						
					}
					else if (!controlStick.GetRawButton(LIFT_BUTTON))
					{
						delay = INITIAL_DELAY;
					}
		}
	/*
	void StepVariable()
	{
		static int delay = INITIAL_DELAY;
		if (leftDriveStick.GetRawButton(9) && delay == INITIAL_DELAY)
		{
		STEPPING_VARIABLE += .005;
		printf("Stepping Varible: %f\n", STEPPING_VARIABLE);
		delay++;
		}
		if (leftDriveStick.GetRawButton(11) && delay == INITIAL_DELAY)
		{
			STEPPING_VARIABLE -= .005;
			printf("Stepping Varible: %f\n", STEPPING_VARIABLE);
			delay++;
		}
		if (leftDriveStick.GetRawButton(10) && delay == INITIAL_DELAY)
		{
			STEPPING_VARIABLE = -.03;
			printf("Stepping Varible: %f\n", STEPPING_VARIABLE);
			delay++;
		}
		else if(!(leftDriveStick.GetRawButton(9) && leftDriveStick.GetRawButton(10) && leftDriveStick.GetRawButton(11)))
		{
			delay = INITIAL_DELAY;
		}
		
	}
	*/
	
	
	
	void MotorBasedGrabber(Joystick &toroStick)
	{
		static float grabberSpeed = GRABBER_STOP;
		static int delay = INITIAL_DELAY;
		
		//ZERO_SPEED_BUTTON is pressed
		if(toroStick.GetRawButton(ZERO_SPEED_BUTTON))
		{
			grabberSpeed = GRABBER_STOP;
		}
		//INCREASE_SPEED_BUTTON pressed but not released
		 else if(delay == INITIAL_DELAY && toroStick.GetRawButton(INCREASE_SPEED_BUTTON))
		{
			if (grabberSpeed + GRABBER_STEP >= GRABBER_MAX_SPEEED_FORWARD)
			{
				grabberSpeed = GRABBER_MAX_SPEEED_FORWARD;
			}
			else
			{
				grabberSpeed = grabberSpeed + GRABBER_STEP;
			}
			delay++;
		}
		//DECREASE_SPEED_BUTTON is pressed but not released
		else if(delay == INITIAL_DELAY && toroStick.GetRawButton(DECREASE_SPEED_BUTTON))
		{
			if (grabberSpeed - GRABBER_STEP <= GRABBER_MAX_SPEEED_REVERSE)
			{
				grabberSpeed = GRABBER_MAX_SPEEED_REVERSE;
			}
			else
			{
				grabberSpeed = grabberSpeed - GRABBER_STEP;
			}		
			delay++;
		}
		//DECREASE_SPEED_BUTTON is released, resets delay
		else if(delay >= INITIAL_DELAY && ! toroStick.GetRawButton(DECREASE_SPEED_BUTTON) && ! toroStick.GetRawButton(INCREASE_SPEED_BUTTON))
		{
			delay = INITIAL_DELAY;
		}
		motorBasedGrabber.Set(grabberSpeed);
		
	}
	
		
		
};

START_ROBOT_CLASS(RobotDemo);

