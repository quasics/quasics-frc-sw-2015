#ifndef MOVE_FOR_DISTANCE_H
#define MOVE_FOR_DISTANCE_H

#include <Commands/Command.h>

class MoveForDistance: public frc::Command {
public:

	MoveForDistance(uint32_t targetInches, double powerLevel);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
private:
	const uint32_t target;
	const double power;
};

#endif
