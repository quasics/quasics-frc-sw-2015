/*#include "boundingRect.h"
 #include "Vision.h"
 #include <iostream>
#include <cmath>

boundingRect::boundingRect() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());

}

// Called just before this Command runs the first time
void boundingRect::Initialize() {
	top = std::max(rectangle1.y, rectangle2.y);

	bottom = std::max(rectangle1.y + rectangle1.height, rectangle2.y + rectangle2.height);

	left = std::max(rectangle1.x, rectangle2.x);

	right = std::max(rectangle1.x + rectangle1.width, rectangle2.x + rectangle2.width);
}

// Called repeatedly when this Command is scheduled to run
void boundingRect::Execute() {
	 if (top = bottom = left = right = 0){
		std::cout<<"Yellow";
	}
}

// Make this return true when this Command no longer needs to run execute()
bool boundingRect::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void boundingRect::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void boundingRect::Interrupted() {

}


*/
