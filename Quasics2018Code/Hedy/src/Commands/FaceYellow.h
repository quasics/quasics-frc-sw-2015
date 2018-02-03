#ifndef FaceYellow_H
#define FaceYellow_H

#include "../CommandBase.h"

class FaceYellow : public CommandBase {
public:
	FaceYellow();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // FaceYellow_H
