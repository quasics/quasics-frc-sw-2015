#include "FaceYellow.h"
#include <iostream>

FaceYellow::FaceYellow() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::driveBase.get());
	Requires(Robot::cubeTracker.get());

}

// Called just before this Command runs the first time
void FaceYellow::Initialize() {
	Robot::driveBase->Stop();
}

// Called repeatedly when this Command is scheduled to run
void FaceYellow::Execute() {
	cv::Rect image, box;
	Robot::cubeTracker->getBoundingRects(image, box);

	int centerBox = box.x + box.width / 2;
	int centerImage = image.width / 2;
	int boxOffsetFromMiddle = centerBox - centerImage;
	const double turningSpeed = .10;

	// If the box is seen within this many pixels of the center, then we're good enough!
	const int allowedOffset = 40;

//	std::cerr << "Image rect: " << image << ", box rect: " << box << std::endl;
//	std::cerr << "   boxOffsetFromMiddle: " << boxOffsetFromMiddle << ", centerImage: " << centerImage << ", centerBox: " << centerBox << std::endl;

	if (box.width == 0) {
		// Note: Come back and revisit this.
		// For now, if we don't see it, don't move.
		Robot::driveBase->Stop();
	}
	else if (boxOffsetFromMiddle > allowedOffset) {
		// Box is to the right of center, so turn right.
		Robot::driveBase->SetPowerToMotors(turningSpeed, turningSpeed);
	}
	else if (boxOffsetFromMiddle < -allowedOffset) {
		// Box is to the left of center, so turn left
		Robot::driveBase->SetPowerToMotors(-turningSpeed, -turningSpeed);
	}
	else {
		Robot::driveBase->Stop();
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
