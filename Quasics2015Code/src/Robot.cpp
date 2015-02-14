#include "Robot.h"

GLaDOS::GLaDOS() :
		driveBase(FrontLeftTalonPort, FrontRightTalonPort, RearLeftTalonPort,
				RearRightTalonPort, LeftEncoderA, LeftEncoderB, RightEncoderA,
				RightEncoderB, GyroIn), powerPad(GamePadIn, 0.05), logicPad(
				GamePad2In, 0.05), camera(CameraHost), elevator(
				ElevatorMotorPort), PissPoorDriveOn(false), LogicSwitchButtonPrevious(
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
}

void GLaDOS::AutonomousPeriodic() {
	//-1
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-1, -1);
	Wait(1);
	printf("-1 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.9
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.9, -.9);
	Wait(1);
	printf("-.9 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.8
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.8, -.8);
	Wait(1);
	printf("-.8 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.7
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.7, -.7);
	Wait(1);
	printf("-.7 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.6
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.6, -.6);
	Wait(1);
	printf("-.6 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.5
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.5, -.5);
	Wait(1);
	printf("-.5 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.4
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.4, -.4);
	Wait(1);
	printf("-.4 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.3
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.3, -.3);
	Wait(1);
	printf("-.3 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.2
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.2, -.2);
	Wait(1);
	printf("-.2 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//-.1
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(-.1, -.1);
	Wait(1);
	printf("-.1 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//0
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(0, 0);
	Wait(1);
	printf("0 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.1
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.1, .1);
	Wait(1);
	printf(".1 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.2
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.2, .2);
	Wait(1);
	printf(".2 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.3
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.3, .3);
	Wait(1);
	printf(".3 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.4
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.4, .4);
	Wait(1);
	printf(".4 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.5
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.5, .5);
	Wait(1);
	printf(".5 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.6
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.6, .6);
	Wait(1);
	printf(".6 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.7
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.7, .7);
	Wait(1);
	printf(".7 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.8
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.8, .8);
	Wait(1);
	printf(".8 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//.9
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(.9, .9);
	Wait(1);
	printf(".9 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));
	//1
	driveBase.ResetSensor(DriveTrain::LeftEncoder);
	driveBase.ResetSensor(DriveTrain::RightEncoder);
	driveBase.SetDrivePower(1, 1);
	Wait(1);
	printf("1 \n Left Encoder: %f \n Right Encoder: %f \n\n",
			driveBase.GetSensorValue(DriveTrain::LeftEncoder),
			driveBase.GetSensorValue(DriveTrain::RightEncoder));

}

void GLaDOS::TeleopInit() {
	printf("GLaDOS Alerts: Initiating Teleoperated Mode\n");
	driveBase.EndDriveAuto();
}

void GLaDOS::TeleopPeriodic() {
	float leftPower = logicPad.GetAxis(Gamepad::LeftStickY) * .5;
	float rightPower = logicPad.GetAxis(Gamepad::RightStickY) * .5;

	driveBase.SetDrivePower(leftPower, rightPower);
	if ((logicPad.GetButton(Gamepad::LeftShoulder)
			|| logicPad.GetButton(Gamepad::RightShoulder))
			&& (logicPad.GetButton(Gamepad::LeftTrigger)
					|| logicPad.GetButton(Gamepad::RightTrigger))) {
		elevator.Off();
	} else if (logicPad.GetButton(Gamepad::LeftShoulder)
			|| logicPad.GetButton(Gamepad::RightShoulder)) {
		elevator.Up();
	} else if (logicPad.GetButton(Gamepad::LeftTrigger)
			|| logicPad.GetButton(Gamepad::RightTrigger)) {
		elevator.Down();
	} else {
		elevator.Off();
	}

	if (LogicSwitchButtonPrevious == true && !logicPad.GetButton(Gamepad::A)) {
		driveBase.SetTrim();
	}
	LogicSwitchButtonPrevious = logicPad.GetButton(Gamepad::A);
}

void GLaDOS::TestPeriodic() {

}

START_ROBOT_CLASS(GLaDOS);

