#ifndef ContinousIntake_H
#define ContinousIntake_H

#include <WPILib.h>

class ContinousIntake : public Command {
public:
	ContinousIntake(double power);
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
	double powerPercent;
	bool isMotorOn;
	bool buttonDown;

};

#endif
