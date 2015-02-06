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
			powerPad.GetAxis(PS4Controller::LeftStickX),
			powerPad.GetAxis(PS4Controller::LeftStickY),
			powerPad.GetAxis(PS4Controller::RightStickX),
			powerPad.GetAxis(PS4Controller::RightStickY));
	Wait (1000);
}

void Robot::TeleopInit() {
	printf("Teleop Start\n");
	driveBase.EndDriveAuto();
}

void Robot::TeleopPeriodic() {
	driveBase.FPSDrive(powerPad.GetAxis(PS4Controller::LeftStickY) * .5,
			powerPad.GetAxis(PS4Controller::RightStickX));
}

void Robot::TestPeriodic() {

}

START_ROBOT_CLASS(Robot);

