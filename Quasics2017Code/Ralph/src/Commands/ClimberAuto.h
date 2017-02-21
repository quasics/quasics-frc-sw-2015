#ifndef ClimberAuto_H
#define ClimberAuto_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class ClimberAuto : public Command {
public:
	ClimberAuto(double power);
	void Initialize() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	double powerPercent;
};

#endif  // ClimberAuto_H
