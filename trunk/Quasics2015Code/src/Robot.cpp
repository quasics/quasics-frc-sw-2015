#include "Robot.h"

GLaDOS::GLaDOS() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), logicPad(
				GamePad2In, 0.05), CameraHost ("10.26.56.11"), camera(CameraHost), elevator(
				ElevatorMotorPort), PissPoorDriveOn(false), LogicSwitchButtonPrevious(
				false), PS4SwitchButtonPrevious(false) {
}

float power;
int testLoop;

void GLaDOS::RobotInit() {
	power = -.95;
	testLoop = 1;
}

void GLaDOS::AutonomousInit() {
	printf("GLaDOS Alerts: Initiating Autonomous Mode\n");
	driveBase.AutoDriveStart(134);
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
	if (logicPad.GetButton(Gamepad::LeftShoulder)
			|| logicPad.GetButton(Gamepad::RightShoulder)) {
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .25,
				logicPad.GetAxis(Gamepad::RightStickY) * .25, leftPower,
				rightPower);
	}
	else{
		driveBase.SmoothStick(logicPad.GetAxis(Gamepad::LeftStickY) * .5,
						logicPad.GetAxis(Gamepad::RightStickY) * .5, leftPower,
						rightPower);
	}
		driveBase.SetDrivePower(leftPower, rightPower);

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
}

START_ROBOT_CLASS(GLaDOS);

