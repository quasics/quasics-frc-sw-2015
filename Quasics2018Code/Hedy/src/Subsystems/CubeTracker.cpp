/*
 * CubeTracker.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: dxc101
 */

#include "CubeTracker.h"

#define IMG_WIDTH	320
#define IMG_HEIGHT	240

CubeTracker::CubeTracker() : frc::Subsystem("CubeTracker") {
	// TODO Auto-generated constructor stub
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
	camera.SetResolution(IMG_WIDTH, IMG_HEIGHT);
	m_lock = new std::mutex;

	visionTrackingTask = new frc::VisionRunner<grip::Vision>(
			camera, new grip::Vision(),
			[&](grip::Vision& pipeline)
			{
				//This code is called each time the pipeline completes. Here we process the results of the pipeline

				//If we have at least 1 contour, we might have a target
 				if (pipeline.GetFilterContoursOutput()->size() > 0)
				{
					int bestArea = 0;
					cv::Rect bestRectangle;
					//Iterate through list of found contours, and find the biggest one.
					for(unsigned int i=0; i < pipeline.GetFilterContoursOutput()->size(); i++)
					{
						cv::Rect rectangle1 = cv::boundingRect(cv::Mat(pipeline.GetFilterContoursOutput()[i]));
						int area = rectangle1.width * rectangle1.height;
						if (area > bestArea) {
							bestArea = area;
							bestRectangle = rectangle1;
						}
					}
					m_lock->lock();
					currentRect = bestRectangle;
					m_lock->unlock();
				}
			});
}

cv::Rect CubeTracker::getCurrentRect() {
	cv::Rect result;
	m_lock->lock();
	result = currentRect;
	m_lock->unlock();
	return result;
}
