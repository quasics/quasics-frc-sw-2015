/*
 * Intake.h
 *
 *  Created on: Jan 20, 2017
 *      Author: axf105
 */

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_

#include "WPILib.h"
#include "../RobotMap.h"

class Intake: public frc::Subsystem {
private:
	std::shared_ptr<SpeedController> intakeMotor;
public:
	Intake();
	virtual ~Intake();
	void TurnOn(double power);
	void TurnOff();
};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
