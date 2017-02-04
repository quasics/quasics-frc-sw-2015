#include "FuelExhaustGate.h"
#include "../RobotMap.h"

#define OPEN_ANGLE		outputActuator->GetMaxAngle()
#define CLOSED_ANGLE	outputActuator->GetMinAngle()

FuelExhaustGate::FuelExhaustGate()
	: Subsystem("FuelExhaustGate"),
	  outputActuator(RobotMap::outputActuator),
	  doorOpen(false)
{
	frc::SmartDashboard::PutData("linearActuator", outputActuator.get());
}


FuelExhaustGate::~FuelExhaustGate() {
	outputActuator = nullptr;
}

void FuelExhaustGate::Set(bool isOpen) {
	doorOpen = isOpen;
	if (isOpen)
		outputActuator->SetAngle(OPEN_ANGLE);
	else
		outputActuator->SetAngle(CLOSED_ANGLE);
}

bool FuelExhaustGate::Get() const {
	return doorOpen;
}

FuelExhaustGate::DoorState FuelExhaustGate::GetDoorStatus() const {
	auto position = outputActuator->GetAngle();
	if (position == OPEN_ANGLE) {
		return eOpen;
	} else if (position == CLOSED_ANGLE) {
		return eClosed;
	} else {
		return eAjar;
	}
}
