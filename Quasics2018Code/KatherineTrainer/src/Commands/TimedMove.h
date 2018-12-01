/*
 * TimedMove.h
 *
 *  Created on: Dec 1, 2018
 *      Author: healym
 */

#ifndef SRC_COMMANDS_TIMEDMOVE_H_
#define SRC_COMMANDS_TIMEDMOVE_H_

#include <Commands/TimedCommand.h>

class TimedMove: public frc::TimedCommand {
private:
	double percentPower_;

public:
	TimedMove(double duration, double percentPower);

	void Initialize() override;
	void End() override;
	void Interrupted() override;
};

#endif /* SRC_COMMANDS_TIMEDMOVE_H_ */
