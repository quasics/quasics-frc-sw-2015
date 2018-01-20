#ifndef UsingGrip_H
#define UsingGrip_H

#include "../CommandBase.h"

class UsingGrip : public CommandBase {
public:
	UsingGrip();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // UsingGrip_H
