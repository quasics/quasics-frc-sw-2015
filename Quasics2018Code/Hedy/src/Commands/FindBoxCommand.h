#ifndef findingBox_H
#define findingBox_H

#include "../Robot.h"
#include "Commands/Subsystem.h"

/**
 * This is a simple command to demonstrate the use of a vision pipeline generated
 * via GRIP, which will report the identified location of one of the power-up cubes
 * (i.e., the largest yellow contour that it finds).
 *
 * Additional work will be needed in order to handle finding the actual vision target
 * for the 2018 game, and moving to it.  But this is a case of "first crawl, then
 * walk, before running".
 */
class FindBoxCommand : public Command {
public:
	FindBoxCommand();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // findingBox_H
