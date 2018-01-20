#ifndef Vision_H
#define Vision_H

#include "../CommandBase.h"

class Vision : public CommandBase {
public:
	Vision();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // Vision_H
