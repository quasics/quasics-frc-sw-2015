/*
 * MoveNorthHeading.h
 *
 *  Created on: Nov 10, 2017
 *      Author: sxm100
 */

#ifndef SRC_COMMANDS_MOVENORTHHEADING_H_
#define SRC_COMMANDS_MOVENORTHHEADING_H_

#include <Commands/Command.h>

class MoveNorthHeading : public frc::Command {
public:
		void Initialize();
		void Execute();
		bool IsFinished();
		void End();
		void Interrupted();
};

#endif /* SRC_COMMANDS_MOVENORTHHEADING_H_ */
