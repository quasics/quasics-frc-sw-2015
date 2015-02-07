#include "Robot.h"

GLaDOS::GLaDOS() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), logicPad(
				GamePad2In, 0.05),/*camera (CameraHost),*/ PissPoorDriveOn(false), LogicSwitchButtonPrevious(
				false), PS4SwitchButtonPrevious(false) {

}

void GLaDOS::RobotInit() {
	printf("Loading GLaDOS: 0/100\n");
	Wait(.1);
	printf("Loading GLaDOS: 10/100\n");
	Wait(.1);
	printf("Loading GLaDOS: 23/100\n");
	Wait(.1);
	printf("Loading GLaDOS: 42/100\n");
	Wait(.1);
	printf("Loading GLaDOS: 50/100\n");
	Wait(.1);
	printf("Loading GLaDOS: 84/100\n");
	Wait(.1);
	printf("Loading GLaDOS: 100/100\n");
	Wait(.1);
	printf("GLaDOS Initialized\n");
	printf("--------------------------\n");
	printf("Morality Core: ");
	Wait(1);
	printf("Acitve\n");
	printf("Inteligence Dampening Core: ");
	Wait(10);
	printf("*ERROR* Core Not Found");
	printf("Cake Core: ");
	Wait(1);
	printf("Acitve\n");
	printf("Fact Core: ");
	Wait(1);
	printf("Acitve\n");
	printf("Space Core: ");
	Wait(1);
	printf("SPAAAAAAAAAAACE\n");
	printf("USB Image Core: ");
	printf("Disabled\n");
	printf("GLaDOS Initalizer Complete \n \n \n");
}

void GLaDOS::AutonomousInit() {
	printf("GLaDOS Alerts: Initiating Autonomous Mode\n");
}

void GLaDOS::AutonomousPeriodic() {

}

void GLaDOS::TeleopInit() {
	printf("GLaDOS Alerts: Initiating Teleoperated Mode\n");
	driveBase.EndDriveAuto();
}

void GLaDOS::TeleopPeriodic() {
	if (PissPoorDriveOn == false) {
		driveBase.FPSDrive(logicPad.GetAxis(Gamepad::LeftStickY) * .5,
				logicPad.GetAxis(Gamepad::RightStickX));
	} else {
		driveBase.FPSDrive(powerPad.GetAxis(PS4Controller::LeftStickY) * .5,
				powerPad.GetAxis(PS4Controller::RightStickX));
	}
	if (PS4SwitchButtonPrevious
			&& !powerPad.GetButton(PS4Controller::Touchpad)) {
		PissPoorDriveOn = true;
	}
	if (LogicSwitchButtonPrevious && !logicPad.GetButton(Gamepad::Back)) {
		PissPoorDriveOn = false;
	}
}

void GLaDOS::TestPeriodic() {

}

START_ROBOT_CLASS(GLaDOS);

