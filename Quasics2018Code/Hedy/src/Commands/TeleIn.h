#ifndef TeleIn_H
#define TeleIn_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class TeleIn : public frc::Command {
public:
	TeleIn();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TeleIn_H
