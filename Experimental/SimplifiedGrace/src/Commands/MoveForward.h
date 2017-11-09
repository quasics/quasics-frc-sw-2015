#ifndef MOVE_FORWARD_H
#define MOVE_FORWARD_H

#include <cstdint>
#include <Commands/Command.h>

class MoveForward : public frc::Command {
public:
	MoveForward(double powerLevel, double seconds);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	const double power;
	const double secondsToRun;
};

#endif  // MOVE_FORWARD_H
