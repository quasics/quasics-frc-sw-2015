#ifndef TURN_LEFT_CONTINUOUS_H
#define TURN_LEFT_CONTINUOUS_H

#include "../CommandBase.h"

class TurnLeftContinuous : public CommandBase {
public:
	TurnLeftContinuous();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TURN_LEFT_CONTINUOUS_H
