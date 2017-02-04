#ifndef SetLightColor_H
#define SetLightColor_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class SetLightColor : public Command {
public:
	SetLightColor();
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
private:

};

#endif  // SetLightColor_H
