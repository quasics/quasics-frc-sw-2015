#ifndef ClimberTeleop_H
#define ClimberTeleop_H

#include "WPILib.h"
#include "../Robot.h"

class ClimberTeleop : public Command {
public:
	ClimberTeleop();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	bool lastButton;
};

#endif  // ClimberTeleop_H
