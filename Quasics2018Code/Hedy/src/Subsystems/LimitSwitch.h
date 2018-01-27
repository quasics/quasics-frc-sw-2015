
#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H
#include "Commands/Subsystem.h"
#include "WPILib.h"


class LimitSwitch: public frc::Subsystem {
private:
	 bool limitSwitch;
	 bool counter;
public:
	LimitSwitch();

	void InitDefaultCommand();
	void InitializeCounter();
	bool CheckSwitch();
	void Stop();
	void Periodic();

};

#endif
