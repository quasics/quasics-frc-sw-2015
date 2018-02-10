/*
 * TurnLightOff.h
 *
 *  Created on: Feb 9, 2018
 *      Author: sth101
 */

#ifndef TURNLIGHTOFF_H_
#define TURNLIGHTOFF_H_

#include <Commands/Command.h>

class TurnLightOff: public frc::Command {
public:
	TurnLightOff();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* TURNLIGHTOFF_H_ */
