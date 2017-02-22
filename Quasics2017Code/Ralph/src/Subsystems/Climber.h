#ifndef Climber_H
#define Climber_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class Climber : public Subsystem {
private:
	std::shared_ptr<SpeedController> climberMotor;

public:
	Climber();
	virtual ~Climber();
	void TurnOn(double power);
	void TurnOff();
};

#endif  // Climber_H
