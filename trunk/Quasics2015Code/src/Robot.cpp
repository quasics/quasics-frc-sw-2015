#include "Robot.h"

GLaDOS::GLaDOS() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), logicPad(
				GamePad2In, 0.05), CameraHost("10.26.56.11"), camera(
				CameraHost), elevator(ElevatorMotorPort), fpsDriveOn(false), DriveSwitchButtonPrevious(
				false) {
}

void GLaDOS::RobotInit() {

}

void GLaDOS::AutonomousInit() {
	driveBase.AutoDriveStart(134);//Set the robot to go forwards for 134 inches
}

void GLaDOS::AutonomousPeriodic() {
	driveBase.AutoProcess();	//Go through the auto process
}

void GLaDOS::TeleopInit() {
	driveBase.EndDriveAuto();	//End auto mode
}

void GLaDOS::TeleopPeriodic() {
	//Drive motor
	float leftPower;	//Create a variable for left motor power
	float rightPower;	//Create a variable for right motor power

	if (logicPad.GetButton(Gamepad::LeftShoulder)
			|| logicPad.GetButton(Gamepad::RightShoulder))//If left or right shoulder is pressed...
	{
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .25,		//Slow stick inputs and trim them
		logicPad.GetAxis(Gamepad::RightStickY) * .25, leftPower, rightPower);
	}
	else //otherwise
	{
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .5,		//Normalize inputs and apply trim
				logicPad.GetAxis(Gamepad::RightStickY) * .5, leftPower,
				rightPower);
	}
	driveBase.SetDrivePower(leftPower, rightPower);	//Tank Drive

	//Elevator Control;
	if (((powerPad.GetButton(Gamepad::LeftShoulder)	//If no buttons pushed or a mixture pushed...
			|| powerPad.GetButton(Gamepad::RightShoulder))
			&& (powerPad.GetButton(Gamepad::LeftTrigger)
					|| powerPad.GetButton(Gamepad::RightTrigger)))
			|| (!powerPad.GetButton(Gamepad::LeftShoulder)
					|| !powerPad.GetButton(Gamepad::RightShoulder)
					|| !powerPad.GetButton(Gamepad::LeftTrigger)
					|| !powerPad.GetButton(Gamepad::RightTrigger))) {
		elevator.Off();	//Do nothing
	} else if (powerPad.GetButton(Gamepad::LeftShoulder)	//Otherwise, if one or both shoulders are pushed...
			|| powerPad.GetButton(Gamepad::RightShoulder)) {
		elevator.Up();	//Lift the elevator
	} else if (powerPad.GetButton(Gamepad::LeftTrigger)	//Otherwise if one or both triggers are pushed...
			|| powerPad.GetButton(Gamepad::RightTrigger)) {
		elevator.Down();	//lower the elevator
	}
}

void GLaDOS::TestPeriodic() {

}

START_ROBOT_CLASS(GLaDOS);

