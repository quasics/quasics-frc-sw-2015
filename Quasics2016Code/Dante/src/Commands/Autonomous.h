#ifndef Autonomous_H
#define Autonomous_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class Autonomous: public Command
{

private:
	int timesMoved;

public:
	Autonomous();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
