#ifndef ArmMoveUp_H
#define ArmMoveUp_H

#include <frc/WPILib.h>

class ArmControl : public frc::Command {
public:
	ArmControl();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ArmMoveUp_H
