#ifndef SetLightColor_H
#define SetLightColor_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class SetLightColor : public Command {
public:
	SetLightColor(Lighting::Colors color = Lighting::kGreen);
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
private:
	Lighting::Colors kColor = Lighting::kGreen;
};

#endif  // SetLightColor_H
