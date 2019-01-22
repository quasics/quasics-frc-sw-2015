///*----------------------------------------------------------------------------*/
///* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
///* Open Source Software - may be modified and shared by FRC teams. The code   */
///* must be accompanied by the FIRST BSD license file in the root directory of */
///* the project.                                                               */
///*----------------------------------------------------------------------------*/
//
//#include "FaceTape.h"
//
//#include <iostream>
//#include <sstream>
//#include <opencv2/core.hpp>
//#include "frc/WPILib.h"
//
//#define ENABLE_DEBUGGING_OUTPUT
//// #define DISABLE_MOTION
//
//
//#ifdef ENABLE_DEBUGGING_OUTPUT
//#define CREATE_SLOG		std::ostringstream sout
//#define SLOG(x)	do { sout << x; } while(0)
//#define LOG(x)	do { std::cerr << x << std::endl; } while(0)
//#define SLOG_DUMP	LOG(sout.str())
//#else
//#define CREATE_SLOG
//#define SLOG(x)
//#define LOG(x)
//#define SLOG_DUMP
//#endif
//
//#include <iostream>
//
//
//FaceTape::FaceTape() {
//	Requires(Robot::driveBase.get());
//	Requires(Robot::tapeTracker.get());
//}
//
//// Called just before this Command runs the first time
//void FaceTape::Initialize() {
//	Robot::driveBase->Stop();
//}
//
//// Called repeatedly when this Command is scheduled to run
//void FaceTape::Execute() {
//	cv::Rect image, box;
//	CREATE_SLOG;
//
//	Robot::tapeTracker->getBoundingRects(image, box);
//	if (box.width == 0) {
//		LOG("   We don't see the target....");
//		Robot::driveBase->Stop();
//		return;
//	}
//
//	int centerRect = box.x + box.width / 2;
//	int centerImage = image.width / 2;
//	int rectOffset = centerRect - centerImage;
//#ifdef DISABLE_MOTION
//	const double rightTurningSpeed = 0;
//	const double leftTurningSpeed = 0;
//#else
//	const double rightTurningSpeed = .15;
//	const double leftTurningSpeed = .1;
//#endif	// DISABLE_MOTION
//
//	// If the box is seen within this many pixels of the center, then we're good enough!
//	const int allowedOffset = 40;
//
//	// LOG("Image rect: " << image << ", box rect: " << box);
//	SLOG("   rectOffsetFromMiddle: " << rectOffset /*<< ", centerImage: " << centerImage << ", center of rectangles: " << centerRect*/ << '\t');
//
//	if (rectOffset > allowedOffset) {
//		SLOG("   Turning right");
//		// Box is to the right of center, so turn right.
//		Robot::driveBase->SetPowerToMotors(rightTurningSpeed, rightTurningSpeed);
//	}
//	else if (rectOffset < -allowedOffset) {
//		SLOG("   Turning left");
//		// Box is to the left of center, so turn left
//		Robot::driveBase->SetPowerToMotors(-leftTurningSpeed, -leftTurningSpeed);
//	}
//	else {
//		SLOG("   Roughly dialed in");
//		//Robot::driveBase->Stop();
//		while(rectOffset == allowedOffset){
//				  SLOG("	Moving Forward");
//				  //Once the box is centered, the robot will begin to move forward
//				  Robot::driveBase->SetPowerToMotors(rightTurningSpeed, leftTurningSpeed);
//			  }
//
//	}
//
//	// Code That I will continue working on to get the robot to move forward once centered
//
//	SLOG_DUMP;
//}
//
//// Make this return true when this Command no longer needs to run execute()
//bool FaceTape::IsFinished() {
//	return false;
//}
//
//// Called once after isFinished returns true
//void FaceTape::End() {
//	Robot::driveBase->Stop();
//}
//
//// Called when another command which requires one or more of the same
//// subsystems is scheduled to run
//void FaceTape::Interrupted() {
//	Robot::driveBase->Stop();
//}
