/*
 * CubeTracker.h
 *
 *  Created on: Jan 27, 2018
 *      Author: dxc101
 */

#ifndef SRC_SUBSYSTEMS_CUBETRACKER_H_
#define SRC_SUBSYSTEMS_CUBETRACKER_H_

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <opencv2/core.hpp>
#include "Vision.h"

class CubeTracker: public frc::Subsystem {
private:
	frc::VisionRunner<grip::Vision> * visionTrackingTask;
	cv::Rect currentRect;
	std::mutex* m_lock;
public:
	CubeTracker();

	cv::Rect getCurrentRect();
};

#endif /* SRC_SUBSYSTEMS_CUBETRACKER_H_ */
