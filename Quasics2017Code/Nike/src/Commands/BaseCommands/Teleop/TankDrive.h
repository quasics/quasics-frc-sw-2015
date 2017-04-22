#ifndef TankDrive_H
#define TankDrive_H

#include "WPILib.h"
#include <Robot.h>
#include <ControllerVariables.h>

class TankDrive : public Command {
public:
	TankDrive();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	bool lastButton;
	bool isReversed;
};

#endif  // TankDrive_H
