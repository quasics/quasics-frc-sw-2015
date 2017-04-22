#include "Climber.h"
#include "../RobotMap.h"

Climber::Climber() :
	Subsystem("Climber") {
	climberMotor = RobotMap::climberMotor;
	climberPower = 0;
	pdb.reset(new PowerDistributionPanel(0));
}

Climber::~Climber() {
	climberMotor = 0;
}

void Climber::TurnOn(double power) {
	climberMotor->Set(power);
	climberPower = power;

}

void Climber::TurnOff() {
	climberMotor->StopMotor();
}

double Climber::GetPower (){
	return climberPower;
}
const uint32_t motorPowerChannel = 5;
double Climber::PrintCurrent(){
	return pdb->GetCurrent(motorPowerChannel);
}
