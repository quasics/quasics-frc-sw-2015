#ifndef ContinousIntake_H
#define ContinousIntake_H

#include "../Robot.h"

class ContinousIntake : public Command {
public:
	ContinousIntake(double powerPercent);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double power;
};

#endif  // ContinousIntake_H
