#ifndef ClimberAuto_H
#define ClimberAuto_H

#include "../Robot.h"

class ClimberAuto : public Command {
public:
	ClimberAuto(double power);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double powerPercent;
};

#endif  // ClimberAuto_H
