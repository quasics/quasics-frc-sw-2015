/*
 * TurnLeft.h
 *
 *  Created on: Feb 1, 2018
 *      Author: sth101
 */

#ifndef SRC_COMMANDS_TURNLEFT_H_
#define SRC_COMMANDS_TURNLEFT_H_

#include <Commands/Command.h>

class TurnLeft : public frc::Command {
public:
	// Makes the turn running motors @ 25% power.
	TurnLeft(double degrees);
	void Initialize() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	const double degreesToTurn;
	double goal;
};
#endif /* SRC_COMMANDS_TURNLEFTINPUT_H_ */

