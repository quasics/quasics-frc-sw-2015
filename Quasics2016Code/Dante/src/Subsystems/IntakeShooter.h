#ifndef IntakeShooter_H
#define IntakeShooter_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class IntakeShooter: public Subsystem
{
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	IntakeShooter();
	void InitDefaultCommand();
};

#endif
