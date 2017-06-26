#include "Robot.h"

void Robot::RobotInit() {
	leftFront.reset(new Jaguar(leftFrontPort));
	leftRear.reset(new Jaguar(leftRearPort));
	rightFront.reset(new Jaguar(rightFrontPort));
	rightRear.reset(new Jaguar(rightRearPort));

	pilotStick.reset(new Joystick(joystickPort));

	if (isLeftReversed) {
		leftFront->SetInverted(true);
		leftRear->SetInverted(true);
		rightFront->SetInverted(false);
		rightRear->SetInverted(false);
	} else {
		leftFront->SetInverted(false);
		leftRear->SetInverted(false);
		rightFront->SetInverted(true);
		rightRear->SetInverted(true);
	}
}

void Robot::TeleopPeriodic() {
	if ((pilotStick->GetRawButton(LeftShoulder)
			|| pilotStick->GetRawButton(RightShoulder))
			&& !(pilotStick->GetRawButton(LeftTrigger)
					|| pilotStick->GetRawButton(RightTrigger))) {
		leftFront->Set(pilotStick->GetRawAxis(leftYAxis) * slowMultiplier);
		leftRear->Set(pilotStick->GetRawAxis(leftYAxis) * slowMultiplier);
		rightFront->Set(pilotStick->GetRawAxis(leftYAxis) * slowMultiplier);
		rightRear->Set(pilotStick->GetRawAxis(leftYAxis) * slowMultiplier);
	} else if (!(pilotStick->GetRawButton(LeftShoulder)
			|| pilotStick->GetRawButton(RightShoulder))
			&& (pilotStick->GetRawButton(LeftTrigger)
					|| pilotStick->GetRawButton(RightTrigger))) {
		leftFront->Set(pilotStick->GetRawAxis(leftYAxis) * turboMultiplier);
		leftRear->Set(pilotStick->GetRawAxis(leftYAxis) * turboMultiplier);
		rightFront->Set(pilotStick->GetRawAxis(leftYAxis) * turboMultiplier);
		rightRear->Set(pilotStick->GetRawAxis(leftYAxis) * turboMultiplier);
	} else {
		leftFront->Set(pilotStick->GetRawAxis(leftYAxis) * middleMultiplier);
		leftRear->Set(pilotStick->GetRawAxis(leftYAxis) * middleMultiplier);
		rightFront->Set(pilotStick->GetRawAxis(leftYAxis) * middleMultiplier);
		rightRear->Set(pilotStick->GetRawAxis(leftYAxis) * middleMultiplier);

	}

}
