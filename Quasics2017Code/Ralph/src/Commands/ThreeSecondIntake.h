#ifndef threeSecondIntake_H
#define threeSecondIntake_H

#include <WPILib.h>

class ThreeSecondIntake : public Command {
public:
	ThreeSecondIntake(double seconds, double power);
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
