#include "FuelExhaustGate.h"
#include "../RobotMap.h"

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
		outputActuator->Set(openValue);
	else
		outputActuator->Set(closeValue);
}

bool FuelExhaustGate::Get() const {
	return doorOpen;
}

FuelExhaustGate::DoorState FuelExhaustGate::GetDoorStatus() const {
	auto position = outputActuator->Get();
	if (position == openValue) {
		return eOpen;
	} else if (position == closeValue) {
		return eClosed;
	} else {
		return eAjar;
	}
}
