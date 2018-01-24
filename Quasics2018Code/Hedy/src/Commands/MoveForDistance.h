

#ifndef MOVE_FOR_DISTANCE_H
#define MOVE_FOR_DISTANCE_H


#include "Commands/Subsystem.h"
#include "../Robot.h"


class MoveForDistance: public frc::Command {
public:

	MoveForDistance(uint32_t targetInches, double powerLevel);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	uint32_t target;
	double power;
};

#endif
