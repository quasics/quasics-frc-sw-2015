#include "UsingGrip.h"
#include "GripPipeline.h"
#include "../Robot.h"

#include <iostream>
#include <cmath>
#include <string>

UsingGrip::UsingGrip() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void UsingGrip::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void UsingGrip::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool UsingGrip::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void UsingGrip::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void UsingGrip::Interrupted() {

}
