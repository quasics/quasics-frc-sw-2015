#ifndef AutoTurnLeft_H
#define AutoTurnLeft_H

#include "../CommandBase.h"
#include "../Robot.h"

class AutoTurnLeft : public CommandBase {
public:
	AutoTurnLeft();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoTurnLeft_H
