#ifndef MoveForDistance_H
#define MoveForDistance_H

#include "WPILib.h"
#include "../../Robot.h"

class MoveForDistance : public Command {
public:
	MoveForDistance(uint32_t targetInches, double powerLevel);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	uint32_t target;
	double power;
};

#endif  // MoveForDistance_H
