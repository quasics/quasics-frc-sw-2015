#ifndef SetLightDynamics_H
#define SetLightDynamics_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class SetLightDynamics : public Command{
public:
	SetLightDynamics(Lighting::Dynamics dynamic = Lighting::kOn);
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
	Lighting::Dynamics kDynamic = Lighting::kOn;

};

#endif  // SetLightDynamics_H
