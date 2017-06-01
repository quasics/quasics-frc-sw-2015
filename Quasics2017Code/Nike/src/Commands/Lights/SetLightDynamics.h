#ifndef SetLightDynamics_H
#define SetLightDynamics_H

#include <Robot.h>
#include "WPILib.h"
#include <Subsystems/ArduinoController.h>

class SetLightDynamics : public Command {
public:
	SetLightDynamics(ArduinoController::BrightnessMode dynamic);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	ArduinoController::BrightnessMode kBrightnessMode;
};

#endif  // SetLightDynamics_H
