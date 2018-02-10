#include "FaceYellow.h"




FaceYellow::FaceYellow() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::driveBase.get());
	Requires(Robot::cubeTracker.get());

}

// Called just before this Command runs the first time
void FaceYellow::Initialize() {
	Robot::driveBase->SetLeftPower(0);
	Robot::driveBase->SetRightPower(0);

}

// Called repeatedly when this Command is scheduled to run
void FaceYellow::Execute() {
	cv::Rect image, box;
	Robot::cubeTracker->getBoundingRects(image, box);
//	image.width
//	box.left
//	box.width

double centerBox = box.width / 2;
double centerImage = image.width / 2;
 if (centerImage > box.x + centerBox)
	{
		Robot::driveBase->SetLeftPower(-.2);
		Robot::driveBase->SetRightPower(.2);
	}
	else if (centerImage < box.x + centerBox)
	{
		Robot::driveBase->SetLeftPower(.2);
		Robot::driveBase->SetRightPower(-.2);
	}
	else
	{
		Robot::driveBase->SetLeftPower(0);
		Robot::driveBase->SetRightPower(0);
	}

}

// Make this return true when this Command no longer needs to run execute()
bool FaceYellow::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void FaceYellow::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FaceYellow::Interrupted() {
	Robot::driveBase->Stop();
}
