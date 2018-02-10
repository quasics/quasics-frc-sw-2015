/*
 * TurnLightOn.h
 *
 *  Created on: Feb 9, 2018
 *      Author: sth101
 */

#ifndef TURNLIGHTON_H_
#define TURNLIGHTON_H_

#include <Commands/Command.h>

class TurnLightOn: public frc::Command {
public:
	TurnLightOn();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* TURNLIGHTON_H_ */
