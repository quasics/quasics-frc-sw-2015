/*
 * CloneableCommand.h
 *
 *  Created on: Feb 20, 2016
 *      Author: healym
 */

#ifndef SRC_COMMANDS_CLONEABLECOMMAND_H_
#define SRC_COMMANDS_CLONEABLECOMMAND_H_

#include <Commands/Command.h>

class CloneableCommand: public Command {
public:
    CloneableCommand() {}
    virtual ~CloneableCommand() {}

    virtual std::unique_ptr<Command> clone() const = 0;
};

#endif /* SRC_COMMANDS_CLONEABLECOMMAND_H_ */
