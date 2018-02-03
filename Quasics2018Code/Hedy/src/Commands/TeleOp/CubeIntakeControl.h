#ifndef TeleIn_H
#define TeleIn_H

#include <WPILib.h>

/**
 * Handles operator input to activate the cube intake (or expulsion) as signaled.
 */
class CubeIntakeControl : public frc::Command {
public:
	CubeIntakeControl();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TeleIn_H
