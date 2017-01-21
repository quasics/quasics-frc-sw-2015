#ifndef threeSecondIntake_H
#define threeSecondIntake_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class threeSecondIntake : public Command {
public:
	threeSecondIntake(double seconds, double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	 double powerPercent;
	 double m_seconds;
	 int counter;
};

#endif  // threeSecondIntake_H
