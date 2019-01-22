/*
 * TurnLightOn.h
 *
 *  Created on: Feb 9, 2018
 *      Author: sth101
 */

#ifndef TURNLIGHTON_H_
#define TURNLIGHTON_H_

#include <frc/commands/Command.h>

class TurnLightOn: public frc::Command {
public:
	TurnLightOn();
	void Initialize();
	bool IsFinished();
};

#endif /* TURNLIGHTON_H_ */
