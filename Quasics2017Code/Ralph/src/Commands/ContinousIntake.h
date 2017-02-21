#ifndef ContinousIntake_H
#define ContinousIntake_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class ContinousIntake : public Command {
public:
	ContinousIntake(double power);
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:
	double powerPercent;
	bool isMotorOn;
	bool buttonDown;
};

#endif
