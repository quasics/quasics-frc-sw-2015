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

private:
	double power;
};

#endif  // AutoTurnLeft_H
