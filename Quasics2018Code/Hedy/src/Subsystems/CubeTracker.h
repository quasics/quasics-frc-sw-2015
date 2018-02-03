/*
 * CubeTracker.h
 *
 *  Created on: Jan 27, 2018
 *      Author: dxc101
 */

#ifndef SRC_SUBSYSTEMS_CUBETRACKER_H_
#define SRC_SUBSYSTEMS_CUBETRACKER_H_

#include <WPILib.h>
#include <opencv2/core.hpp>
#include "Vision.h"

/**
 * This is a sample subsystem, developed to find one of the yellow "power up"
 * cubes from the 2018 FRC game.  The vision pipeline it is designed to work
 */
class CubeTracker: public frc::Subsystem {
private:
	frc::VisionRunner<grip::Vision> * visionTrackingTask;
	cv::Rect currentRect;
	cv::Rect imageRect;
	mutable std::mutex* m_lock;
	std::thread* m_visionThread;
public:
	CubeTracker();
	void visionExecuter();

	cv::Rect getCurrentRect() const;
	cv::Rect getImageRect() const;
};

#endif /* SRC_SUBSYSTEMS_CUBETRACKER_H_ */
