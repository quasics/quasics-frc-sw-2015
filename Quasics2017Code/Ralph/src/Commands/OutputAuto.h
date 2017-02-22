#ifndef OutputAuto_H
#define OutputAuto_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class OutputAuto : public Command {
public:
	OutputAuto(double power);

	void Initialize() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	double powerPercent;
};

#endif  // OutputAuto_H
