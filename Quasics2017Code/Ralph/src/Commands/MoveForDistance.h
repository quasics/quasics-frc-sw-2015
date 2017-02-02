#ifndef MoveForDistance_H
#define MoveForDistance_H

#include "../CommandBase.h"

class MoveForDistance : public CommandBase {
public:
	MoveForDistance();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // MoveForDistance_H
