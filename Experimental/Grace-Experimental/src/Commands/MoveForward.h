#ifndef MOVE_FORWARD_H
#define MOVE_FORWARD_H

#include <cstdint>
#include <Commands/Command.h>

// TODO: We should *seriously* consider rewriting this to use the WPILib's
// "frc::TimedCommand" class as a base, rather than re-rolling time management
// for ourselves.
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
