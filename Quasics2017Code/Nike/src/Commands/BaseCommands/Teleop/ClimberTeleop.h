#ifndef ClimberTeleop_H
#define ClimberTeleop_H

#include "WPILib.h"
#include <Robot.h>
#include <ControllerVariables.h>

class ClimberTeleop : public Command {
public:
	ClimberTeleop();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool previousButton;
	bool isOn;
};

#endif  // ClimberTeleop_H
