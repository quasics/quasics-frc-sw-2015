#ifndef TeleOut_H
#define TeleOut_H

#include <WPILib.h>

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
