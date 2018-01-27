#ifndef findingBox_H
#define findingBox_H

#include "../Robot.h"
#include "Commands/Subsystem.h"

class findingBox : public Command {
public:
	findingBox();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	//ds
};

#endif  // findingBox_H
