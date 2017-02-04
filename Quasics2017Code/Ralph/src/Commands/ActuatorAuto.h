#ifndef ActuatorAuto_H
#define ActuatorAuto_H

#include "WPILib.h"
#include "../Subsystems/FuelExhaustGate.h"

class ActuatorAuto : public Command {
public:
	ActuatorAuto(bool doorOpen);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	const bool openDoor;

};

#endif  // ActuatorAuto_H
