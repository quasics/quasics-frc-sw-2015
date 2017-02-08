#ifndef SetLightColor_H
#define SetLightColor_H

#include <WPILib.h>

class SetLightColor : public Command {
public:
	SetLightColor();
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

#endif  // SetLightColor_H
