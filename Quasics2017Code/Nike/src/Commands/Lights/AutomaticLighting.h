#ifndef AutomaticLighting_H
#define AutomaticLighting_H

#include "Robot.h"
#include "WPILib.h"
#include <Subsystems/ArduinoController.h>

class AutomaticLighting : public Command {
public:
	AutomaticLighting();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	ArduinoController::ColorMode color;
	ArduinoController::BrightnessMode dynamic;
	uint32_t batteryTimer;
};

#endif  // AutomaticLighting_H
