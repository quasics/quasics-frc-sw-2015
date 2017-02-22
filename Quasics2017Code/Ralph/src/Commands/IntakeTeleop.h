#ifndef IntakeTeleop_H
#define IntakeTeleop_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class IntakeTeleop : public Command {
public:
	IntakeTeleop(double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double powerPercent;
	bool buttonPrev;
	bool motorOn;
};

#endif  // IntakeTeleop_H
