#ifndef MoveForwardTime_H
#define MoveForwardTime_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class MoveForwardTime: public Command
{
private:
	double m_seconds;
	double m_power;
	int counter;
public:

	MoveForwardTime(double seconds, double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
