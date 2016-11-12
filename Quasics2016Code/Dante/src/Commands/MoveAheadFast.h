#ifndef MoveAheadFast_H
#define MoveAheadFast_H

#include "Commands/Command.h"
#include "Commands/Subsystem.h"
#include "WPILib.h"

class MoveAheadFast: public Command
{
private:
	int timesMoved;
	int counter;
	double m_seconds;
	double m_power;
	int timer;
	int stopTime;



public:
	MoveAheadFast();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
