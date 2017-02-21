#include "ActuatorAuto.h"

#include "../Robot.h"
#include "../Subsystems/FuelExhaustGate.h"

ActuatorAuto::ActuatorAuto(bool doorOpen) : openDoor(doorOpen) {
	// Use Requires() here to declare subsystem dependencies
	// assert(Robot::fuelExhaustGate.get());
	Requires(Robot::fuelExhaustGate.get());
}

// Called just before this Command runs the first time
void ActuatorAuto::Initialize() {
	Robot::fuelExhaustGate->Set(openDoor);
}


// Make this return true when this Command no longer needs to run execute()
bool ActuatorAuto::IsFinished() {
	// TODO: Fix this.  The command isn't finished untl the actuator is either
	// fully extended or fully retracted.  Otherwise, we run the risk of
	// having balls jam into place if the door isn't open yet, or come
	// out unexpectedly if it isn't closed yet.
	return true;
}

void ActuatorAuto::Execute() {
	printf("Actuator position: %lf\n",
			Robot::fuelExhaustGate->GetPosition());
}
