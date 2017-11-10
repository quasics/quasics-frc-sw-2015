#ifndef TURN_LEFT_CONTINUOUS_H
#define TURN_LEFT_CONTINUOUS_H

#include <Commands/Command.h>

class TurnLeftContinuous : public frc::Command {
public:
	TurnLeftContinuous();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TURN_LEFT_CONTINUOUS_H
