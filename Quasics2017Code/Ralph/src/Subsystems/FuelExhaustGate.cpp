#include "FuelExhaustGate.h"
#include "../RobotMap.h"

FuelExhaustGate::FuelExhaustGate() : Subsystem("ExampleSubsystem") {
	outputActuator = RobotMap::gearServo;
		doorOpen = false;
}


FuelExhaustGate::~FuelExhaustGate() {
	outputActuator = 0;
}

void FuelExhaustGate::Set(bool isOpen) {
	doorOpen = isOpen;
	if (isOpen)
		outputActuator->Set(openValue);
	else
		outputActuator->Set(closeValue);
}

bool FuelExhaustGate::Get() {
	return doorOpen;
}
