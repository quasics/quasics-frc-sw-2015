#ifndef GearDoorTeleop_H
#define GearDoorTeleop_H

#include "WPILib.h"
#include <Robot.h>
#include <ControllerVariables.h>

class GearDoorTeleop : public Command {
public:
	GearDoorTeleop();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	bool previousButton;
	bool previousKickerButton;
};

#endif  // GearDoorTeleop_H
