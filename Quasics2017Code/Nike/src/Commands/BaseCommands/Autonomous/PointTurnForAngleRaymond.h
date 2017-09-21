#ifndef PointTurnForAngle_H
#define PointTurnForAngle_H

#include <WPILib.h>
#include "../../../Robot.h"

class PointTurnForAngle : public Command {
public:
	PointTurnForAngle(double targetDegreesAntiClockwise, double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

	double userPower;
	double targetDegrees;
	bool isAntiClockwise;
};

#endif  // PointTurnForAngle_H
