#include "Robot.h"
std::unique_ptr<SpeedController> driveBaseleftFront;
std::unique_ptr<SpeedController> driveBaseleftRear;
std::unique_ptr<SpeedController> driveBaserightFront;
std::unique_ptr<SpeedController> driveBaserightRear;
std::unique_ptr<Joystick> pilotStick;

void Robot::RobotInit() {
	driveBaseleftFront.reset(new Jaguar(0));
	driveBaseleftRear.reset(new Jaguar(1));
	driveBaserightFront.reset(new Jaguar(2));
	driveBaserightRear.reset(new Jaguar(3));
	pilotStick.reset(new Joystick(0));

	driveBaseleftFront->SetInverted(true);
	driveBaseleftRear->SetInverted(true);
}

const unsigned char leftYAxis = 1;
const unsigned char rightYAxis = 3;
const double multiplier = -0.65;

void Robot::TeleopPeriodic() {
	driveBaseleftFront->Set(pilotStick->GetRawAxis(leftYAxis) * multiplier);
	driveBaseleftRear->Set(pilotStick->GetRawAxis(leftYAxis) * multiplier);
	driveBaserightFront->Set(pilotStick->GetRawAxis(leftYAxis) * multiplier);
	driveBaserightRear->Set(pilotStick->GetRawAxis(leftYAxis) * multiplier);
}
