#ifndef Climber_H
#define Climber_H

#include <WPILib.h>

// CODE_REVIEW(mjh): Document what this class actually does.
class Climber : public Subsystem {
private:
	std::shared_ptr<SpeedController> climberMotor;
	std::shared_ptr<PowerDistributionPanel> pdb;
	double climberPower;

public:
	Climber();
	virtual ~Climber();
	void TurnOn(double power);
	void TurnOff();
	double PrintCurrent();
	double GetPower ();
};

#endif  // Climber_H
