#include "Robot.h"

Robot::Robot() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05) {

}

void Robot::RobotInit() {

}

void Robot::AutonomousInit() {

}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
	driveBase.SetDrivePower(powerPad.GetAxis(Gamepad::LeftStickY), powerPad.GetAxis(Gamepad::RightStickY));

}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot);

