#ifndef MoveForward2_H
#define MoveForward2_H

#include <Commands/Command.h>

class MoveForward2: public Command
{

private:
	double m_seconds;
	double m_power;
	int counter;

public:
	MoveForward2(double seconds, double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
