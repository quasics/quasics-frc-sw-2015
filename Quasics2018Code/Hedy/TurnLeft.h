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
	TurnLeft(double degrees);
	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

private:
	const double DegreesToTurn;
	double Goal;
};
#endif /* SRC_COMMANDS_TURNLEFTINPUT_H_ */

