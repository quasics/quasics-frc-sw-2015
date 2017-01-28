#ifndef IntakeAuto_H
#define IntakeAuto_H

#include "../Robot.h"

class IntakeAuto : public Command {
public:
	IntakeAuto(double power);
	virtual void Initialize();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
	double powerPercent;


};

#endif  // IntakeAuto_H
