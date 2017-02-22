#ifndef SetLightColor_H
#define SetLightColor_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
// CODE_REVIEW(mjh): Is this class actually required?  (It's currently a set of null-ops.)
class SetLightColor : public Command {
public:
	SetLightColor();
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;
};

#endif  // SetLightColor_H
