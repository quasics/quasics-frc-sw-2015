#ifndef PixyAutoFineTuning_H
#define PixyAutoFineTuning_H

#include "WPILib.h"
#include "../RobotVariables.h"

class PixyAutoFineTuning: public Command {
public:
	PixyAutoFineTuning();
	void Initialize();
	void Execute();bool IsFinished();
	void End();
	void Interrupted();
private:
	uint32_t timer;
	bool isFarLeft;
	bool isAligned;
	bool isTooFar;
};

#endif  // PixyAutoFineTuning_H
