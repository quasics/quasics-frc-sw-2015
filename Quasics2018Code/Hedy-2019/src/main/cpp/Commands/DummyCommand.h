#ifndef DUMMY_COMMAND_H
#define DUMMY_COMMAND_H

#include "frc/commands/Subsystem.h"
#include "../Robot.h"

/**
 * Used to create placeholder entities in the Smart Dashboard UI
 * (e.g., for option selection).  This command should never actually
 * be executed on the bot.
 */
class DummyCommand: public frc::Command {
public:
	DummyCommand() {}
	bool IsFinished() override { return true; }
};

#endif
