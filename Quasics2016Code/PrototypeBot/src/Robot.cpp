#include "Robot.h"
#include <iostream>

ProtoMan::ProtoMan() :
		stick(0), left(4), right(5), leftEncoder(4, 5), rightEncoder(6, 7, true) {
	leftEncoder.SetDistancePerPulse(1);
	rightEncoder.SetDistancePerPulse(1);
}

void ProtoMan::RobotInit() {
	leftEncoder.Reset();
	rightEncoder.Reset();
}

void ProtoMan::AutonomousInit() {
	leftEncoder.Reset();
	rightEncoder.Reset();
}

void ProtoMan::AutonomousPeriodic() {

}
void ProtoMan::TeleopInit() {
	std::cout << "Axis Controlled Intake" << std::endl;
	leftEncoder.Reset();
	rightEncoder.Reset();
}
void ProtoMan::TeleopPeriodic() {
	float leftPower = -stick.GetRawAxis(1);
	float rightPower = -stick.GetRawAxis(1);

	if (leftPower >= 0) {
		if (std::abs(leftEncoder.GetRaw() - rightEncoder.GetRaw()) <= 5) {
			std::cout << "Balanced" << std::endl;
			left.Set(leftPower * .75);
			right.Set(-rightPower * .75);
		} else if (fabs(leftEncoder.Get()) > fabs(rightEncoder.Get())) {
			std::cout << "- Right" << std::endl;
			rightPower = rightPower - .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		} else {
			std::cout << "- Left" << std::endl;
			leftPower = leftPower - .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		}
	} else {
		if (std::abs(leftEncoder.GetRaw() - rightEncoder.GetRaw()) <= 5) {
			left.Set(leftPower * .75);
			right.Set(-rightPower * .75);
		} else if (fabs(leftEncoder.Get()) < fabs(rightEncoder.Get())) {
			std::cout << "+ Right" << std::endl;
			rightPower = rightPower + .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		} else {
			std::cout << "+ Left" << std::endl;
			leftPower = leftPower + .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		}
	}

	std::cout << "Left: " << leftEncoder.Get() << std::endl << "Left Power: "
			<< leftPower << std::endl << "Left Rate: " << leftEncoder.GetRate()
			<< std::endl;
	std::cout << "Right: " << rightEncoder.Get() << std::endl << "Right Power: "
			<< rightPower << std::endl << "Right Rate: "
			<< rightEncoder.GetRate() << std::endl << std::endl;
}
void ProtoMan::TestInit() {
	std::cout << "Button Controlled Intake" << std::endl;
	leftEncoder.Reset();
	rightEncoder.Reset();
}
void ProtoMan::TestPeriodic() {

	float leftPower = 0;
	float rightPower = 0;

	if (stick.GetRawButton(3) && !stick.GetRawButton(2))
		leftPower = .5;
	else if (stick.GetRawButton(2) && !stick.GetRawButton(3))
		leftPower = -.5;
	else
		leftPower = 0;
	rightPower = leftPower;


	if (leftPower >= 0) {
		if (std::abs(leftEncoder.GetRaw() - rightEncoder.GetRaw()) <= 5) {
			std::cout << "Balanced" << std::endl;
			left.Set(leftPower * .75);
			right.Set(-rightPower * .75);
		} else if (fabs(leftEncoder.Get()) > fabs(rightEncoder.Get())) {
			std::cout << "- Right" << std::endl;
			rightPower = rightPower - .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		} else {
			std::cout << "- Left" << std::endl;
			leftPower = leftPower - .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		}
	} else {
		if (std::abs(leftEncoder.GetRaw() - rightEncoder.GetRaw()) <= 5) {
			left.Set(leftPower * .75);
			right.Set(-rightPower * .75);
		} else if (fabs(leftEncoder.Get()) < fabs(rightEncoder.Get())) {
			std::cout << "+ Right" << std::endl;
			rightPower = rightPower + .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		} else {
			std::cout << "+ Left" << std::endl;
			leftPower = leftPower + .25;
			left.Set((leftPower) * .75);
			right.Set(-(rightPower) * .75);
		}
	}

	std::cout << "Left: " << leftEncoder.Get() << std::endl << "Left Power: "
			<< leftPower << std::endl << "Left Rate: " << leftEncoder.GetRate()
			<< std::endl;
	std::cout << "Right: " << rightEncoder.Get() << std::endl << "Right Power: "
			<< rightPower << std::endl << "Right Rate: "
			<< rightEncoder.GetRate() << std::endl << std::endl;
}

START_ROBOT_CLASS(ProtoMan);
