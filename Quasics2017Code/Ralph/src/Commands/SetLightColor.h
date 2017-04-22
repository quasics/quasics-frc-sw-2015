#ifndef SetLightColor_H
#define SetLightColor_H

#include <WPILib.h>
#include "../Robot.h"

// CODE_REVIEW(mjh): Document what this class actually does.
// CODE_REVIEW(mjh): Is this class actually required?  (It's currently a set of null-ops.)
class SetLightColor : public Command {
public:
	SetLightColor(Arduino::ColorMode mode);
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:
	Arduino::ColorMode kMode;
};

#endif  // SetLightColor_H
