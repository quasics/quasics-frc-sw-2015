#ifndef MoveAheadFast_H
#define MoveAheadFast_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class MoveAheadFast: public Command
{
private:
	int timesMoved;
public:
	MoveAheadFast();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
