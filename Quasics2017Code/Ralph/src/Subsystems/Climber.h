#ifndef Climber_H
#define Climber_H

#include "WPILib.h"
#include "../RobotMap.h"

class Climber : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	std::shared_ptr<SpeedController> climberMotor;

public:
	Climber();
	virtual ~Climber();
	void TurnOn(double power);
	void TurnOff();
};

#endif  // Climber_H
