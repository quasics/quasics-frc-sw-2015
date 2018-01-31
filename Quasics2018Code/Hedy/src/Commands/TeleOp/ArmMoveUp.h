#ifndef ArmMoveUp_H
#define ArmMoveUp_H

#include <WPILib.h>

class ArmMoveUp : public frc::Command {
public:
	ArmMoveUp();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ArmMoveUp_H
