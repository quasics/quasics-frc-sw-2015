#include "findingBox.h"
#include "Vision.h"

findingBox::findingBox() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::cubeTracker.get());
}

// Called just before this Command runs the first time
void findingBox::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void findingBox::Execute() {

		std::cout<<"Yellow Box Found!" << std::endl;

}

// Make this return true when this Command no longer needs to run execute()
bool findingBox::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void findingBox::End() {
//ds
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void findingBox::Interrupted() {
}
