#ifndef MoveForward_H
#define MoveForward_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class MoveForward: public Command
{
public:
	MoveForward();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	int counter;

};

#endif
