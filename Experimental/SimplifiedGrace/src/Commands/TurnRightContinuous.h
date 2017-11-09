#ifndef TURN_RIGHT_CONTINUOUS_H
#define TURN_RIGHT_CONTINUOUS_H

#include <Commands/Command.h>

class TurnRightContinuous: public frc::Command {
public:
	TurnRightContinuous();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* TURN_RIGHT_CONTINUOUS_H */
