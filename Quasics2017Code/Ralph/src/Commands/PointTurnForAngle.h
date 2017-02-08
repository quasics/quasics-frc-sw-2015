#ifndef PointTurnForAngle_H
#define PointTurnForAngle_H

#include <WPILib.h>

class PointTurnForAngle : public Command {
public:
	PointTurnForAngle(double targetDegreesAntiClockwise, double powerPercent);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double targetDegrees;
	double power;
};

#endif  // PointTurnForAngle_H
