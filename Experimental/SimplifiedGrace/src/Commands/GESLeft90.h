#ifndef GES_LEFT_90_H
#define GES_LEFT_90_H

#include <Commands/Command.h>

class GESLeft90 : public frc::Command {
public:
	GESLeft90();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // GES_LEFT90_H
