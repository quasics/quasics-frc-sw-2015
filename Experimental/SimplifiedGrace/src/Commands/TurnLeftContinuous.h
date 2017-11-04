#ifndef AutoTurnLeft_H
#define AutoTurnLeft_H

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

#endif  // AutoTurnLeft_H
