#include "Robot.h"

	Robot::Robot() :
	leftFront (FrontLeftTalonPort),
	leftRear (RearLeftTalonPort),
	rightFront (FrontRightTalonPort),
	rightRear (RearRightTalonPort),
	powerPad (GamePadIn,6,10)

	{

	}

	void Robot::RobotInit() {
		leftFront.Set (0);
		leftRear.Set (0);
		rightFront.Set (0);
		rightRear.Set (0);
	}

	void Robot::AutonomousInit() {
		leftFront.Set (0);
		leftRear.Set (0);
		rightFront.Set (0);
		rightRear.Set (0);
	}

	void Robot::AutonomousPeriodic() {

	}

	void Robot::TeleopInit() {
		leftFront.Set (0);
		leftRear.Set (0);
		rightFront.Set (0);
		rightRear.Set (0);
	}

	void Robot::TeleopPeriodic() {
		float leftPower = powerPad.GetRawAxis(2);
		float rightPower = powerPad.GetRawAxis(5);
		float multiplierNow;

		if(powerPad.GetRawButton(5) == true || powerPad.GetRawButton(6) == true){
			multiplierNow = SlowMultiplier;
		}
		else if(powerPad.GetRawAxis(3) < 0 || powerPad.GetRawAxis(3) > 0){
			multiplierNow = TurboMultiplier;
		}
		else{
			multiplierNow = NormalMultiplier;
		}

		if (fabs(leftPower) <= DeadbandWidth){
			leftPower = 0;
		}
		if (fabs(rightPower) <= DeadbandWidth){
			rightPower = 0;
		}

		leftFront.Set (leftPower * multiplierNow);
		leftRear.Set (leftPower  * multiplierNow);
		rightFront.Set (rightPower * multiplierNow);
		rightRear.Set (rightPower * multiplierNow);
	}

	void Robot::TestPeriodic() {

	}

	START_ROBOT_CLASS(Robot);


