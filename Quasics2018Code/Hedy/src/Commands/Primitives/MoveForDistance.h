#ifndef MOVE_FOR_DISTANCE_H
#define MOVE_FOR_DISTANCE_H

#include <Commands/Command.h>

inline uint32_t feetToInches(double feet) { return uint32_t(feet * 12); }

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
