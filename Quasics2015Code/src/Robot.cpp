#include "Robot.h"

GLaDOS::GLaDOS() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), logicPad(
				GamePad2In, 0.05), CameraHost("10.26.56.11"), camera(
				CameraHost), elevator(ElevatorMotorPort), PissPoorDriveOn(
				false), LogicSwitchButtonPrevious(false), PS4SwitchButtonPrevious(
				false) {
}

float power;
int testLoop;

int autoCounter;

void GLaDOS::RobotInit() {
	power = -.95;
	testLoop = 1;
}

void GLaDOS::AutonomousInit() {
	printf("GLaDOS Alerts: Initiating Autonomous Mode\n");
	//driveBase.AutoDriveStart(134);
	autoCounter = 0;
}

void GLaDOS::AutonomousPeriodic() {
	//driveBase.AutoProcess();
	if (autoCounter < 50){
		driveBase.SetDrivePower(.5, .5);
		autoCounter = autoCounter +1;
	}
	else{
		driveBase.SetDrivePower(0,0);
	}
}

void GLaDOS::TeleopInit() {
	printf("GLaDOS Alerts: Initiating Teleoperated Mode\n");
	//driveBase.EndDriveAuto();
}

void GLaDOS::TeleopPeriodic() {
	float leftPower;
	float rightPower;
	if ((logicPad.GetButton(Gamepad::LeftShoulder)
			|| logicPad.GetButton(Gamepad::RightShoulder))
			&& (logicPad.GetButton(Gamepad::LeftTrigger)
					|| logicPad.GetButton(Gamepad::RightTrigger))) {
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .4,
				logicPad.GetAxis(Gamepad::RightStickY) *.4, leftPower,
				rightPower);
	} else if (logicPad.GetButton(Gamepad::RightShoulder)
			|| logicPad.GetButton(Gamepad::LeftShoulder)) {
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .3,
				logicPad.GetAxis(Gamepad::RightStickY) * .3, leftPower,
				rightPower);
	} else if (logicPad.GetButton(Gamepad::RightTrigger)
			|| logicPad.GetButton(Gamepad::LeftShoulder)) {
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .5,
				logicPad.GetAxis(Gamepad::RightStickY) * .5, leftPower,
				rightPower);
	} else {
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .4,
				logicPad.GetAxis(Gamepad::RightStickY) * .4, leftPower,
				rightPower);
	}
	driveBase.SetDrivePower(leftPower, rightPower);

	if ((powerPad.GetButton(Gamepad::LeftShoulder)
			|| powerPad.GetButton(Gamepad::RightShoulder))
			&& (powerPad.GetButton(Gamepad::LeftTrigger)
					|| powerPad.GetButton(Gamepad::RightTrigger))) {
		elevator.Off();
	} else if (powerPad.GetButton(Gamepad::LeftShoulder) || powerPad.GetButton(Gamepad::RightShoulder)) {
		elevator.Up();
	} else if (powerPad.GetButton(Gamepad::LeftTrigger) || powerPad.GetButton(Gamepad::RightTrigger)) {
		elevator.Down();
	} else {
		elevator.Off();
	}
}

void GLaDOS::TestInit() {
	driveBase.TrimTest(-1);
	driveBase.TrimTest(-.95);
	driveBase.TrimTest(-.9);
	driveBase.TrimTest(-.85);
	driveBase.TrimTest(-.8);
	driveBase.TrimTest(-.75);
	driveBase.TrimTest(-.7);
	driveBase.TrimTest(-.65);
	driveBase.TrimTest(-.6);
	driveBase.TrimTest(-.55);
	driveBase.TrimTest(-.5);
	driveBase.TrimTest(-.45);
	driveBase.TrimTest(-.4);
	driveBase.TrimTest(-.35);
	driveBase.TrimTest(-.3);
	driveBase.TrimTest(-.25);
	driveBase.TrimTest(-.2);
	driveBase.TrimTest(-.15);
	driveBase.TrimTest(-.1);
	driveBase.TrimTest(0);
	driveBase.TrimTest(.1);
	driveBase.TrimTest(.15);
	driveBase.TrimTest(.2);
	driveBase.TrimTest(.25);
	driveBase.TrimTest(.3);
	driveBase.TrimTest(.35);
	driveBase.TrimTest(.4);
	driveBase.TrimTest(.45);
	driveBase.TrimTest(.5);
	driveBase.TrimTest(.55);
	driveBase.TrimTest(.6);
	driveBase.TrimTest(.65);
	driveBase.TrimTest(.7);
	driveBase.TrimTest(.75);
	driveBase.TrimTest(.8);
	driveBase.TrimTest(.85);
	driveBase.TrimTest(.9);
	driveBase.TrimTest(.95);
	driveBase.TrimTest(1);
}

void GLaDOS::TestPeriodic() {

}

START_ROBOT_CLASS(GLaDOS);

