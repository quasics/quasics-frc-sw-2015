#include <Commands/FindBoxCommand.h>

FindBoxCommand::FindBoxCommand() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::cubeTracker.get());
}

// Called just before this Command runs the first time
void FindBoxCommand::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void FindBoxCommand::Execute() {

		std::cout<<"Yellow box data: " << Robot::cubeTracker->getCurrentRect() << std::endl;

}

// Make this return true when this Command no longer needs to run execute()
bool FindBoxCommand::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void FindBoxCommand::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FindBoxCommand::Interrupted() {
}
