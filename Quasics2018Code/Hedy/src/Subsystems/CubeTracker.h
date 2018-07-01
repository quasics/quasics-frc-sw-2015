/*
 * This file declares a simple vision tracking subsystem, which will
 * attempt to identify one of the yellow "power up" cubes from the
 * 2018 game.  It was developed as a prototyping exercise during the
 * 2018 Build Season, handling image data from the on-board (USB) camera,
 * and processed on the Rio.
 *
 * It is preserved here as a possible source of example code in future
 * efforts.
 */

//#ifndef SRC_SUBSYSTEMS_CUBETRACKER_H_
//#define SRC_SUBSYSTEMS_CUBETRACKER_H_
//
//#include <WPILib.h>
//#include <opencv2/core.hpp>
//#include "Vision.h"
//
///**
// * This is a sample subsystem, developed to find one of the yellow "power up"
// * cubes from the 2018 FRC game.  The vision pipeline it is designed to work
// * with ("Vision.h") is generated from the "CubeTracker.grip" definition.
// */
//class CubeTracker: public frc::Subsystem {
//private:
//	frc::VisionRunner<grip::Vision> * visionTrackingTask;
//	cv::Rect currentRect;
//	cv::Rect imageRect;
//	mutable std::mutex* m_lock;
//	std::thread* m_visionThread;
//
//public:
//	CubeTracker();
//	void visionExecuter();
//
//	cv::Rect getCurrentRect() const;
//	cv::Rect getImageRect() const;
//
//	void getBoundingRects(cv::Rect& imageRect, cv::Rect& currentRect) const;
//};
//
//#endif /* SRC_SUBSYSTEMS_CUBETRACKER_H_ */
