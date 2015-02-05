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
	printf(
			"Left Stick X: %f \n Left Stick Y: %f \n Right Stick X: %f \n Right Stick Y: %f \n",
			powerPad.GetAxis(Gamepad::LeftStickX),
			powerPad.GetAxis(Gamepad::LeftStickY),
			powerPad.GetAxis(Gamepad::RightStickX),
			powerPad.GetAxis(Gamepad::RightStickY));
	Wait (1000);
}

void Robot::TeleopInit() {
	printf("Teleop Start\n");
	driveBase.EndDriveAuto();
}

void Robot::TeleopPeriodic() {
	driveBase.FPSDrive(powerPad.GetAxis(Gamepad::LeftStickY) * .5,
			powerPad.GetAxis(Gamepad::RightStickX));
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot);

