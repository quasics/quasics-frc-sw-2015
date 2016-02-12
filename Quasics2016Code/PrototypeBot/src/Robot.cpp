#include "Robot.h"
#include <iostream>

ProtoMan::ProtoMan() :
		stick(0), left(4), right(5), leftEncoder(3, 4), rightEncoder(5, 6, true) {
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
	std::cout <<"Button Controlled Intake";
	leftEncoder.Reset();
	rightEncoder.Reset();
}
void ProtoMan::TeleopPeriodic() {
	float power = stick.GetRawAxis(1);

	if (std::abs(leftEncoder.GetRaw() - rightEncoder.GetRaw()) <= 5) {
		left.Set(power * .5);
		right.Set(-power * .5);
	} else if (leftEncoder.GetRaw() > rightEncoder.GetRaw()) {
		std::cout <<"Slowing Left" <<std::endl;
		left.Set((power - .125) * .5);
		right.Set(-(power) * .5);
	} else {
		std::cout <<"Slowing Right" <<std::endl;
		left.Set((power) * .5);
		right.Set(-(power - .125) * .5);
	}

}
void ProtoMan::TestInit() {
	leftEncoder.Reset();
	rightEncoder.Reset();
}
void ProtoMan::TestPeriodic() {

	float power = 0;
	if (stick.GetRawButton(3) && !stick.GetRawButton(2))
		power = .5;
	else if (stick.GetRawButton(2) && !stick.GetRawButton(3))
		power = -.5;
	else
		power = 0;

	if (std::abs(leftEncoder.GetRaw() - rightEncoder.GetRaw()) <= 5) {
		left.Set(power * .5);
		right.Set(-power * .5);
	} else if (leftEncoder.GetRaw() > rightEncoder.GetRaw()) {
		left.Set((power - .125) * .5);
		right.Set(-(power) * .5);
	} else {
		left.Set((power) * .5);
		right.Set(-(power - .125) * .5);
	}
}

START_ROBOT_CLASS(ProtoMan);
