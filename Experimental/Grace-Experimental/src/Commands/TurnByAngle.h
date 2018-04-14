/*
 * TurnByAngle.h
 *
 *  Created on: Apr 14, 2018
 *      Author: healym
 */

#ifndef SRC_COMMANDS_TURNBYANGLE_H_
#define SRC_COMMANDS_TURNBYANGLE_H_

#include <Commands/Command.h>

class TurnByAngle: public frc::Command {
private:
	double angleDegrees;
public:
	TurnByAngle(double angleDegrees);

	void Initialize() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;
};

#endif /* SRC_COMMANDS_TURNBYANGLE_H_ */
