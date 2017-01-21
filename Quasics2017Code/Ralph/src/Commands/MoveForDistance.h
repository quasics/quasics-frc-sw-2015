#ifndef MoveForDistance_H
#define MoveForDistance_H

#include "../Robot.h"

class MoveForDistance : public Command {
public:
	MoveForDistance();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // MoveForDistance_H
