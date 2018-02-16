#include "TapeTracker.h"

// Standard C++ headers
#include <iostream>
#include <mutex>
#include <thread>

// WPILib headers
#include <CameraServer.h>
#include <Commands/Subsystem.h>
#include <vision/VisionRunner.h>


#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define WIDTH_SCALING		.7
#define HEIGHT_SCALING		.7

TapeTracker::TapeTracker() : frc::Subsystem("TapeTracker") {
/*
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(IMG_WIDTH, IMG_HEIGHT);
		m_lock = new std::mutex;

		visionTrackingTask = new frc::VisionRunner<grip::Vision>(
				camera, new grip::Vision(),
				[&](grip::Vision& pipeline)
				{
					// Remember how big the source image was.
					cv::Rect srcRect;
					srcRect.x = 0;
					srcRect.y = 0;
					srcRect.width = IMG_WIDTH * WIDTH_SCALING;
					srcRect.height = IMG_HEIGHT * HEIGHT_SCALING;

					//If we have at least 2 contours, we might have a target
					cv::Rect bestRectangle;
					const auto & filterCountoursOutput = *pipeline.GetFilterContoursOutput();
	 				if (filterCountoursOutput.size() > 1)
					{
						int bestScore = 0;
						//Iterate through list of found contours.
						for(unsigned int i=0; i < filterCountoursOutput.size(); i++)
						{
							const std::vector<cv::Point> & countourPoints = filterCountoursOutput[i];
							const cv::Rect rectangle1 = cv::boundingRect(cv::Mat(countourPoints));

							for(unsigned int i=0; i < findCountoursOutput.size(); i++){



							}


							// TODO: Add code to look at pairs of spotted contours, and find
							// the best pair.  Then, save the bounding box for the pair in
							// "bestRectangle".
							//code that is assumed to be used is down below
						}


					}
					m_lock->lock();
					currentRect = bestRectangle;
					imageRect = srcRect;
					m_lock->unlock();
				});

*/
}




/*
 * cv::Mat findContoursInput = cvErodeOutput;
	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	//Step Filter_Contours0:
	//input
	std::vector<std::vector<cv::Point> > filterContoursContours = findContoursOutput;
	double filterContoursMinArea = 20.0;  // default Double
	double filterContoursMinPerimeter = 20.0;  // default Double
	double filterContoursMinWidth = 10.0;  // default Double
	double filterContoursMaxWidth = 1000.0;  // default Double
	double filterContoursMinHeight = 10.0;  // default Double
	double filterContoursMaxHeight = 1000.0;  // default Double
	double filterContoursSolidity[] = {0, 100};
	double filterContoursMaxVertices = 1000000.0;  // default Double
	double filterContoursMinVertices = 0.0;  // default Double
	double filterContoursMinRatio = 0.0;  // default Double
	double filterContoursMaxRatio = 1000.0;  // default Double
	filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, this->filterContoursOutput);


 */

