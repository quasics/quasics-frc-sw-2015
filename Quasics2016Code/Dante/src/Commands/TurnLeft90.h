#ifndef TurnLeft90_H
#define TurnLeft90_H

#include <Commands/Command.h>

class TurnLeft90: public Command
{

private:
	double m_seconds;
	int counter;
	double m_power;

public:
	TurnLeft90(double seconds, double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
