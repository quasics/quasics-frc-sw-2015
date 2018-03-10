/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "FaceTape.h"

#include <iostream>
#include <opencv2/core.hpp>
#include "WPILib.h"

#define ENABLE_DEBUGGING_OUTPUT
// #define DISABLE_MOTION


#ifdef ENABLE_DEBUGGING_OUTPUT
#define LOG(x)	do { std::cerr << x << std::endl; } while(0)
#else
#define LOG(x)
#endif

#ifndef VISION_TRACK_CUBES
#include <iostream>


FaceTape::FaceTape() {

	Requires(Robot::driveBase.get());
	Requires(Robot::tapeTracker.get());
	//Requires(Robot::tapeTracker.get());
}

// Called just before this Command runs the first time
void FaceTape::Initialize() {
	Robot::driveBase->Stop();
}

// Called repeatedly when this Command is scheduled to run
void FaceTape::Execute() {

	cv::Rect image, box;
	Robot::tapeTracker->getBoundingRects(image, box);
	LOG("Image rect: " << image << ", box rect: " << box);

	if (box.width == 0) {

		LOG("   We don't see the rectangles....");
		Robot::driveBase->Stop();
		return;
	}

	int centerRect = box.x + box.width / 2;
	int centerImage = image.width / 2;
	int rectOffset = centerRect - centerImage;
#ifdef DISABLE_MOTION
	const double turningSpeed = 0;
#else
	const double turningSpeed = .2;
#endif	// DISABLE_MOTION

	// If the box is seen within this many pixels of the center, then we're good enough!
	const int allowedOffset = 15;

	LOG("   rectOffsetFromMiddle: " << rectOffset << ", centerImage: " << centerImage << ", center of rectangles: " << centerRect);

	if (rectOffset > allowedOffset) {
		LOG("   Turning right");
		// Box is to the right of center, so turn right.
		Robot::driveBase->SetPowerToMotors(-turningSpeed, -turningSpeed);
	}
	else if (rectOffset < -allowedOffset) {
		LOG("   Turning left");
		// Box is to the left of center, so turn left
		Robot::driveBase->SetPowerToMotors(turningSpeed, turningSpeed);
	}
	else {
		LOG("   Roughly dialed in");
		Robot::driveBase->Stop();
	}

}

// Make this return true when this Command no longer needs to run execute()
bool FaceTape::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void FaceTape::End() {
	Robot::driveBase->Stop();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FaceTape::Interrupted() {
	Robot::driveBase->Stop();
}

#endif	// VISION_TRACK_CUBES
