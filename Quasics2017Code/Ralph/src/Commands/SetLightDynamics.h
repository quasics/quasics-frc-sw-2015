#ifndef SetLightDynamics_H
#define SetLightDynamics_H

#include <WPILib.h>

class SetLightDynamics : public Command {
public:
	SetLightDynamics();
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

#endif  // SetLightDynamics_H
