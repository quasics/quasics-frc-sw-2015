#ifndef Outtake_H
#define Outtake_H

#include "WPILib.h"
#include "../RobotMap.h"

class Outtake : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	std::shared_ptr<Spark> outputMotor;
public:
	Outtake();
	void TurnOn(double power);
	void TurnOff();
};

#endif  // Outtake_H
