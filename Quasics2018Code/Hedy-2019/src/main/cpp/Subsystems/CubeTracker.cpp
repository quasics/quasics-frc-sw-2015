/*
 * This file defines a simple vision tracking subsystem, which will
 * attempt to identify one of the yellow "power up" cubes from the
 * 2018 game.  It was developed as a prototyping exercise during the
 * 2018 Build Season, handling image data from the on-board (USB) camera,
 * and processed on the Rio.
 *
 * It is preserved here as a possible source of example code in future
 * efforts.
 */

//#include "CubeTracker.h"
//
//// Standard C++ headers
//#include <iostream>
//#include <mutex>
//#include <thread>
//
//// WPILib headers
//#include <CameraServer.h>
//#include <Commands/Subsystem.h>
//#include <vision/VisionRunner.h>
//
//#define IMG_WIDTH	320
//#define IMG_HEIGHT	240
//
//// Note: the following values mirror those that are currently hard-coded into Vision.cpp,
//// based on the GRIP pipeline configuration.  If we change the sizing in the pipeline,
//// then we'll need to update the values here, too.
//#define WIDTH_SCALING		.7
//#define HEIGHT_SCALING		.7
//
//CubeTracker::CubeTracker() : frc::Subsystem("CubeTracker") {
//	/*
//	 * The following code is based on an example provided at:
//	 * https://github.com/wpilibsuite/roboRIOVisionExamples/tree/master/2018/2018VisionSampleCPP
//	 *
//	 * It was then modified to:
//	 * a) Fix a bug on line 79 of the original code (where a "*" is missing).
//	 * b) Look for the single largest bounding box, since the sample pipeline that
//	 *    we're currently using it with is trying to find one of the yellow game
//	 *    cubes, rather than looking for the 2 reflective tape stripes marking a
//	 *    point on the Switch in the 2018 FRC game.
//	 */
//	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
//	camera.SetResolution(IMG_WIDTH, IMG_HEIGHT);
//	m_lock = new std::mutex;
//
//	visionTrackingTask = new frc::VisionRunner<grip::Vision>(
//			camera, new grip::Vision(),
//			[&](grip::Vision& pipeline)
//			{
//				// Remember how big the source image was.
//				cv::Rect srcRect;
//				srcRect.x = 0;
//				srcRect.y = 0;
//				srcRect.width = IMG_WIDTH * WIDTH_SCALING;
//				srcRect.height = IMG_HEIGHT * HEIGHT_SCALING;
//
//				//If we have at least 1 contour, we might have a target
//				cv::Rect bestRectangle;
//				const auto & filterCountoursOutput = *pipeline.GetFilterContoursOutput();
// 				if (filterCountoursOutput.size() > 0)
//				{
//					int bestArea = 0;
//					//Iterate through list of found contours, and find the biggest one.
//					for(unsigned int i=0; i < filterCountoursOutput.size(); i++)
//					{
//						const std::vector<cv::Point> & countourPoints = filterCountoursOutput[i];
//						const cv::Rect rectangle1 = cv::boundingRect(cv::Mat(countourPoints));
//						const int area = rectangle1.width * rectangle1.height;
//						if (area > bestArea) {
//							bestArea = area;
//							bestRectangle = rectangle1;
//						}
//					}
//				}
//				m_lock->lock();
//				currentRect = bestRectangle;
//				imageRect = srcRect;
//				m_lock->unlock();
//			});
//	m_visionThread = new std::thread(&CubeTracker::visionExecuter, this);
//}
//
//void CubeTracker::visionExecuter()
//{
//	std::cerr << "Starting up vision execution" << std::endl;
//	visionTrackingTask->RunForever();
//}
//
//void CubeTracker::getBoundingRects(cv::Rect& imageRect, cv::Rect& currentRect) const {
//	m_lock->lock();
//	imageRect = this->imageRect;
//	currentRect = this->currentRect;
//	m_lock->unlock();
//}
//
//cv::Rect CubeTracker::getImageRect() const {
//	cv::Rect imageRect, ignored;
//	getBoundingRects(imageRect, ignored);
//	return imageRect;
//}
//
//cv::Rect CubeTracker::getCurrentRect() const {
//	cv::Rect currentRect, ignored;
//	getBoundingRects(ignored, currentRect);
//	return imageRect;
//
//	cv::Rect result;
//	m_lock->lock();
//	result = currentRect;
//	m_lock->unlock();
//	return result;
//}
