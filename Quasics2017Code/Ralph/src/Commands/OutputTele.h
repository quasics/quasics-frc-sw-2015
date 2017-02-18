#ifndef OutputTele_H
#define OutputTele_H

#include <WPILib.h>

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
	bool buttonPrev;
	bool motorOn;
};

#endif  // OutputTele_H
