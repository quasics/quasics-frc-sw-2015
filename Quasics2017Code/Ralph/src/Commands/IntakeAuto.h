#ifndef IntakeAuto_H
#define IntakeAuto_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class IntakeAuto : public Command {
public:
	IntakeAuto(double power);

	virtual void Initialize() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:
	double powerPercent;
};

#endif  // IntakeAuto_H
