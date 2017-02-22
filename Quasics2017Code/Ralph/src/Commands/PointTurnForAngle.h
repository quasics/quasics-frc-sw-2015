#ifndef PointTurnForAngle_H
#define PointTurnForAngle_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class PointTurnForAngle : public Command {
public:
	PointTurnForAngle(double targetDegreesAntiClockwise, double powerPercent);

	void Initialize() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	double targetDegrees;
	double power;
};

#endif  // PointTurnForAngle_H
