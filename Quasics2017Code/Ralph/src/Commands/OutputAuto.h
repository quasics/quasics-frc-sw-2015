#ifndef OutputAuto_H
#define OutputAuto_H

#include "../Robot.h"
#include "../Subsystems/Outtake.h"

class OutputAuto : public Command {
public:
	OutputAuto(double power);
	void Initialize();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double powerPercent;
};

#endif  // OutputAuto_H
