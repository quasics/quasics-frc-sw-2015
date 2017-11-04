#ifndef moveForward_H
#define moveForward_H

#include "../CommandBase.h"
#include <cstdint>

// TODO: We should *seriously* consider rewriting this to use the WPILib's
// "frc::TimedCommand" class as a base, rather than re-rolling time management
// for ourselves.
class MoveForward : public CommandBase {
public:
	MoveForward(double powerLevel, double seconds);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	const double power;
	const uint32_t cyclesToRun;
	uint32_t counter;
};

#endif  // moveForward_H
