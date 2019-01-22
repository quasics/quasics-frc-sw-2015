/*
 * LeftTurn.h
 *
 *  Created on: Dec 1, 2018
 *      Author: Developer
 */

#ifndef SRC_COMMANDS_LEFTTURN_H_
#define SRC_COMMANDS_LEFTTURN_H_

#include <frc/commands/TimedCommand.h>

class LeftTurn: public frc::TimedCommand {
private:
	double percentPower_;
public:
	LeftTurn(double duration, double percentPower);
	void Initialize() override;
		void End() override;
		void Interrupted() override;
};

#endif /* SRC_COMMANDS_LEFTTURN_H_ */
