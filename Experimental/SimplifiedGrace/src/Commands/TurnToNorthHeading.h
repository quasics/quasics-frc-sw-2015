/*
 * TurnToCompassHeading.h
 *
 *  Created on: Nov 10, 2017
 *      Author: sxm100
 */

#ifndef SRC_COMMANDS_TURNTONORTHHEADING_H_
#define SRC_COMMANDS_TURNTONORTHHEADING_H_

#include <Commands/Command.h>



class TurnToNorthHeading : public frc::Command {
public:
	TurnToNorthHeading();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

private:
	double heading;
	bool wasTurningToLeft;
};

#endif /* SRC_COMMANDS_TURNTONORTHHEADING_H_ */
