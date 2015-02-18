#include "Robot.h"

GLaDOS::GLaDOS() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), logicPad(
				GamePad2In, 0.05), camera(CameraHost), elevator(
				ElevatorMotorPort), PissPoorDriveOn(false), LogicSwitchButtonPrevious(
				false), PS4SwitchButtonPrevious(false) {

}

float power;
int testLoop;

void GLaDOS::RobotInit() {
	power = -.95;
	testLoop = 1;
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
	printf("*ERROR* Core Not Found\n");
	printf("Cake Core: ");
	Wait(1);
	printf("Acitve\n");
	printf("Fact Core: ");
	Wait(1);
	printf("Acitve\n");
	printf("Space Core: ");
	Wait(1);
	printf("SPAAAAAAAAAAACE\n");
	printf("Ethernet Image Processor: ");
	printf("Disabled\n");
	printf("GLaDOS Initalizer Complete \n \n \n");
}

void GLaDOS::AutonomousInit() {
	printf("GLaDOS Alerts: Initiating Autonomous Mode\n");
	driveBase.AutoDriveStart(24);
}

void GLaDOS::AutonomousPeriodic() {
	driveBase.AutoProcess();
}

void GLaDOS::TeleopInit() {
	printf("GLaDOS Alerts: Initiating Teleoperated Mode\n");
	driveBase.EndDriveAuto();
}

void GLaDOS::TeleopPeriodic() {
	float leftPower;
	float rightPower;

	driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY)* .25, logicPad.GetAxis(Gamepad::RightStickY)* .25, leftPower, rightPower);
	driveBase.SetDrivePower(leftPower , rightPower );

	if ((powerPad.GetButton(Gamepad::LeftShoulder)
			|| powerPad.GetButton(Gamepad::RightShoulder))
			&& (powerPad.GetButton(Gamepad::LeftTrigger)
					|| powerPad.GetButton(Gamepad::RightTrigger))) {
		elevator.Off();
	} else if (powerPad.GetButton(Gamepad::LeftShoulder)
			|| powerPad.GetButton(Gamepad::RightShoulder)) {
		elevator.Up();
	} else if (powerPad.GetButton(Gamepad::LeftTrigger)
			|| powerPad.GetButton(Gamepad::RightTrigger)) {
		elevator.Down();
	} else {
		elevator.Off();
	}
}

void GLaDOS::TestPeriodic() {
	driveBase.TrimTest(power);
	power = power + .1;
	testLoop = testLoop + 1;
	while (testLoop >= 21){
		driveBase.SetDrivePower (0,0);
	}
}

START_ROBOT_CLASS(GLaDOS);

