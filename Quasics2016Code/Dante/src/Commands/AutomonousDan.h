#ifndef AutomonousDan_H
#define AutomonousDan_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class AutomonousDan: public Command
{

private:
	int timesMoved;

public:
	AutomonousDan();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
