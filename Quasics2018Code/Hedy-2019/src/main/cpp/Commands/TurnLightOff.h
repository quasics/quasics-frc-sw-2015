/*
 * TurnLightOn.h
 *
 *  Created on: Feb 9, 2018
 *      Author: sth101
 */

#ifndef TURNLIGHTOFF_H_
#define TURNLIGHTOFF_H_

#include <frc/commands/Command.h>

class TurnLightOff: public frc::Command {
public:
	TurnLightOff();
	void Initialize();
	bool IsFinished();
};

#endif /* TURNLIGHTOFF_H_ */
