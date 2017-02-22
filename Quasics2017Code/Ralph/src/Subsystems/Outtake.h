#ifndef Outtake_H
#define Outtake_H

#include <WPILib.h>

// CODE_REVIEW(mjh): This #include should be in the .cpp file, since it's not needed by clients.
#include "../RobotMap.h"

// CODE_REVIEW(mjh): Document what this class actually does.
class Outtake : public Subsystem {
private:
	std::shared_ptr<Spark> outputMotor;

public:
	Outtake();
	void TurnOn(double power);
	void TurnOff();
};

#endif  // Outtake_H
