#ifndef Default_H
#define Default_H

#include "../CommandBase.h"
#include "WPILib.h"

class Default: public CommandBase
{
public:
	Default();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
