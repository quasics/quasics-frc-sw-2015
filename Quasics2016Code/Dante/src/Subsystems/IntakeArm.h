#ifndef IntakeArm_H
#define IntakeArm_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class IntakeArm: public Subsystem
{
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	IntakeArm();
	void InitDefaultCommand();
};

#endif
