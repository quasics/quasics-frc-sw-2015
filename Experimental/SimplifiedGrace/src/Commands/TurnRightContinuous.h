#ifndef TURN_RIGHT_CONTINUOUS_H
#define TURN_RIGHT_CONTINUOUS_H

#include "../CommandBase.h"

class TurnRightContinuous: public CommandBase {
public:
	TurnRightContinuous();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* TURN_RIGHT_CONTINUOUS_H */
