#ifndef AutoTurnLeft_H
#define AutoTurnLeft_H

#include "WPILib.h"
#include <Robot.h>

class AutoTurnLeft : public Command {

private:
	double m_seconds;
	double power;
	int counter;
public:
	AutoTurnLeft(double seconds, double powerLevel);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoTurnLeft_H
