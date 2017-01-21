#ifndef SwingTurnForAngle_H
#define SwingTurnForAngle_H

#include "../Robot.h"

class SwingTurnForAngle : public Command {
public:
	SwingTurnForAngle(double targetDegreesAntiClockwise, double powerPercent);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
double targetDegrees;
double power;

};


#endif  // SwingTurnForAngle_H
