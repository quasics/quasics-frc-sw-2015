#ifndef ReverseIntakeAuto_H
#define ReverseIntakeAuto_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class ReverseIntakeAuto : public Command {
public:
	ReverseIntakeAuto(double power);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double powerPercent;
};

#endif  // ReverseIntakeAuto_H
