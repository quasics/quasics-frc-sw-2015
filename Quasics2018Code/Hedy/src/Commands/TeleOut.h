#ifndef TeleOut_H
#define TeleOut_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class TeleOut : public frc::Command {
public:
	TeleOut();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TeleOut_H
