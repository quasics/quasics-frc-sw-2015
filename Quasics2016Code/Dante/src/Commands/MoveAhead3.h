#ifndef MoveAhead3_H
#define MoveAhead3_H

#include <Commands/Command.h>

class MoveAhead3: public Command {

private:
	double m_seconds;
	double m_power;
	int counter;


public:
	MoveAhead3(double seconds, double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
