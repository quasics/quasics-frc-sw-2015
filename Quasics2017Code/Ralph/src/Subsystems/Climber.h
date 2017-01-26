/*
 * Climber.h
 *
 *  Created on: Jan 24, 2017
 *      Author: axf105
 */

#ifndef SRC_SUBSYSTEMS_CLIMBER_H_
#define SRC_SUBSYSTEMS_CLIMBER_H_

#include <Commands/Subsystem.h>
#include "WPILib.h"

class Climber: public frc::Subsystem {
private:
	std::shared_ptr<SpeedController> climberMotor;
public:
	Climber();
	virtual ~Climber();
	void TurnOn(double power);
	void TurnOff();
};

#endif /* SRC_SUBSYSTEMS_CLIMBER_H_ */
