#ifndef FaceYellow_H
#define FaceYellow_H

#include "../Robot.h"
#include "Commands/Subsystem.h"

class FaceYellow : public Command {
public:
	FaceYellow();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // FaceYellow_H
