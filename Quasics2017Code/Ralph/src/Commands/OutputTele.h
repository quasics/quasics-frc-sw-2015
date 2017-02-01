#ifndef OutputTele_H
#define OutputTele_H

#include "../Robot.h"

class OutputTele : public Command {
public:
	OutputTele(double power);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double powerPercent;
	bool motorOn;
	bool buttonDown;
};

#endif  // OutputTele_H
