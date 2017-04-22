#ifndef SetLightDynamics_H
#define SetLightDynamics_H

#include <WPILib.h>
#include "Robot.h"

// CODE_REVIEW(mjh): Document what this class actually does.
// CODE_REVIEW(mjh): Is this class actually required?  (It's currently a set of null-ops.)
class SetLightDynamics : public Command {
public:
	SetLightDynamics(Arduino::BrightnessMode mode);

	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:
	Arduino::BrightnessMode kMode;
};

#endif  // SetLightDynamics_H
