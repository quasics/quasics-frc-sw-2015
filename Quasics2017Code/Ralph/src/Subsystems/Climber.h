/*
 * Climber.h
 *
 *  Created on: Jan 24, 2017
 *      Author: axf105
 */

#ifndef SRC_SUBSYSTEMS_CLIMBER_H_
#define SRC_SUBSYSTEMS_CLIMBER_H_

#include "../Robot.h"

class Climber: public Command {
public:
	Climber(double power);
virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();

private:
	double powerPercent;
	bool isMotorOn;
	bool buttonDown;
};

#endif /* SRC_SUBSYSTEMS_CLIMBER_H_ */
