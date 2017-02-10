#include "FuelExhaustGate.h"
#include "../RobotMap.h"
#include "../RobotMap.h"

#define OPEN_VAL		1
#define CLOSED_VAL		0.19

FuelExhaustGate::FuelExhaustGate() :
		Subsystem("FuelExhaustGate"), outputActuator(RobotMap::outputActuator), doorOpen(
				false) {
}

FuelExhaustGate::~FuelExhaustGate() {
	outputActuator = nullptr;
}

void FuelExhaustGate::Set(bool isOpen) {
	doorOpen = isOpen;
	if (isOpen)
		outputActuator->Set(OPEN_VAL);
	else
		outputActuator->Set(CLOSED_VAL);
}

bool FuelExhaustGate::Get() const {
	return doorOpen;
}

FuelExhaustGate::DoorState FuelExhaustGate::GetDoorStatus() const {
	auto position = outputActuator->Get();
	if (position == OPEN_VAL) {
		return eOpen;
	} else if (position == CLOSED_VAL) {
		return eClosed;
	} else {
		return eAjar;
	}
}
