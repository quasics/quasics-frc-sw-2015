#include "Robot.h"

Robot::Robot() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), camera(
				CameraHost), elevator(LeftElevatorMotorPort,
				RightElevatorMotorPort), FPSDriveOff(false), StartButtonPrevious(
				false) {

}

void Robot::RobotInit() {
	printf("Robot Init\n");
}

void Robot::AutonomousInit() {
	printf("Auto Start\n");
}

void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
	printf("Teleop Start\n");
	driveBase.EndDriveAuto();
}

void Robot::TeleopPeriodic() {
	if (powerPad.GetButton(Gamepad::Start) == false
			&& StartButtonPrevious == true) {
		FPSDriveOff = !FPSDriveOff;
	}
	StartButtonPrevious = powerPad.GetButton(Gamepad::Start);

	if (FPSDriveOff == false) {
		driveBase.FPSDrive(powerPad.GetAxis(Gamepad::LeftStickY) * .5,
				powerPad.GetAxis(Gamepad::RightStickX));
	} else {
		driveBase.SetDrivePower(powerPad.GetAxis(Gamepad::LeftStickY) * .5,
				powerPad.GetAxis(Gamepad::RightStickY) * .5);
	}
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot);

