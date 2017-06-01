#ifndef SetLightColor_H
#define SetLightColor_H

#include <Robot.h>
#include "WPILib.h"
#include <Subsystems/ArduinoController.h>

class SetLightColor : public Command {
public:
	SetLightColor(ArduinoController::ColorMode);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	ArduinoController::ColorMode kColorMode;
};

#endif  // SetLightColor_H
