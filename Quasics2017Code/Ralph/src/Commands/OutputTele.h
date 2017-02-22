#ifndef OutputTele_H
#define OutputTele_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class OutputTele : public Command {
public:
	OutputTele(double power);

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	double powerPercent;
	bool buttonPrev;
	bool motorOn;
};

#endif  // OutputTele_H
