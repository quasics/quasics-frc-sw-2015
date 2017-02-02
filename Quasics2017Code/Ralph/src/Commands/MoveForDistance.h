#ifndef MoveForDistance_H
#define MoveForDistance_H

#include "WPILib.h"
#include "../Robot.h"

class MoveForDistance : public Command {
public:
	MoveForDistance(int targetInches, float powerMagnitude);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	int target;
	float power;
};

#endif  // MoveForDistance_H
