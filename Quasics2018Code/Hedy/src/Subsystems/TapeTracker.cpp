/*
 * This file defines a simple vision tracking subsystem, which will
 * attempt to identify the area on the "switch" from the 2018 game
 * where power cubes are to be delivered during the autonomous round,
 * using the photo-reflective tape markers on them, and a high-brightness
 * green LED ring mounted around the camera, in turn mounted on the robot.
 *
 * It was developed as a prototyping exercise during the 2018 Build Season,
 * handling image data from the on-board (USB) camera, which is processed
 * on the driver's station and then communicated back to the Rio via a
 * NetworkTable.  (The "GreenSeeking.grip" file defines the GRIP pipeline
 * to be executed on the driver's station.)
 *
 * It is preserved here as a possible source of example code in future
 * efforts.
 */

#include "TapeTracker.h"
#include "WPILib.h"
#include "NetworkTables/NetworkTable.h"

// Standard C++ headers
#include <iostream>
#include <mutex>
#include <thread>
#include <cmath>

// WPILib headers
#include <CameraServer.h>
#include <Commands/Subsystem.h>
#include <vision/VisionRunner.h>

#ifdef USE_NATIVE_RESOLUTION
// Natural resolution: 720p (1280 x 720)
#define IMG_WIDTH 1280
#define IMG_HEIGHT 720
#else
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#endif

#define ENABLE_EXTERNAL_PIPELINE

#define WIDTH_SCALING		.7
#define HEIGHT_SCALING		.7

// #define ENABLE_DEBUGGING_OUTPUT

// Helper class, used in evaluating pairs of rectangles for scoring.
class boundingRect
{
	public:
		int top;
		int bottom;
		int left;
		int right;

	boundingRect (cv::Rect rectangle1, cv::Rect rectangle2)
	{
		top = std::max(rectangle1.y, rectangle2.y);
		bottom = std::max(rectangle1.y + rectangle1.height, rectangle2.y + rectangle2.height);
		left = std::max(rectangle1.x, rectangle2.x);
		right = std::max(rectangle1.x + rectangle1.width, rectangle2.x + rectangle2.width);
	}

	boundingRect()
	{
		top = bottom = left = right = 0;
	}
};



 /* Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
 */
double ratioToScore(double ratio)
{
	return (std::max(0.0, std::min(100*(1-std::abs(1-ratio)), 100.0)));
}

//The height of the bounding box around both rectangles should be approximately double the width
double boundingRatioScore(cv::Rect rectangle1, cv::Rect rectangle2)
{
	boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

	return ratioToScore((bounding->top-bounding->bottom)/(2*(bounding->left-bounding->right)));
}

//The width of either contour should be approximately 1/4 of the total bounding box width
double contourWidthScore(cv::Rect rectangle1, cv::Rect rectangle2)
{
	boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

	return ratioToScore(rectangle1.width*4/(bounding->right-bounding->left));
}

//The top edges should be very close together. Find the difference, then scale it by the bounding box height.
//This results in an ideal 0 instead of an ideal 1, so add 1
double topEdgeScore(cv::Rect rectangle1, cv::Rect rectangle2)
{
	boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

	return ratioToScore(1 + (rectangle1.y - rectangle2.y)/(bounding->top-bounding->bottom));
}

//The spacing between the left edges should be 3/4 of the target width
double leftSpacingScore(cv::Rect rectangle1, cv::Rect rectangle2)
{
	boundingRect* bounding = new boundingRect(rectangle1, rectangle2);

	return ratioToScore(std::abs(rectangle2.x - rectangle1.x)*3/(4*bounding->right-bounding->left));
}

//The width of the two contours should match
double widthRatioScore(cv::Rect rectangle1, cv::Rect rectangle2)
{
	return ratioToScore(rectangle1.width/rectangle2.width);
}

//The height of the two contours should match
double heightRatioScore(cv::Rect rectangle1, cv::Rect rectangle2)
{
	return ratioToScore(rectangle1.height/rectangle2.height);
}

TapeTracker::TapeTracker()
: frc::Subsystem("TapeTracker"),
  usbCamera(new cs::UsbCamera(frc::CameraServer::GetInstance()->StartAutomaticCapture())),
  m_lock(new std::mutex),
  table(NetworkTable::GetTable("GRIP/ContourReport"))
{
	usbCamera->SetResolution(IMG_WIDTH, IMG_HEIGHT);
	m_visionThread = new std::thread(&TapeTracker::visionExecuter, this);
}

void TapeTracker::processDriverStationData(cv::Rect& bestRectangle) {
	bestRectangle = cv::Rect();		// Clear it to all zero values.

	const std::vector<double> centerXs = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	const std::vector<double> centerYs = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	const std::vector<double> widths = table->GetNumberArray("width", llvm::ArrayRef<double>());
	const std::vector<double> heights = table->GetNumberArray("height", llvm::ArrayRef<double>());

	// Note that we'll assume all arrays are of the same size (and nothing changes while we
	// were grabbing them).
	if (centerXs.empty()) {
		// N	o contours were apparently seen
#ifdef ENABLE_DEBUGGING_OUTPUT
		std::cout << "Didn't receive any rects from GRIP" << std::endl;
#endif	// ENABLE_DEBUGGING_OUTPUT
	}
	else{
#ifdef ENABLE_DEBUGGING_OUTPUT
		std::cout << "Received " << centerXs.size() << " rects from GRIP" << std::endl;
#endif	// ENABLE_DEBUGGING_OUTPUTdasfj
	}

	// Process the data from the network table, in order to fill in "bestRectangle".
	/*
	 * If we have > 1 thing (centerX, or centerY, ....) coming back
	 *    1) Build a vector of cv::Rect objects using the data from GRIP.
	 *    2) Loop through the vector we built (start @ 0, go to size - 2); call this "i"
	 *       a) Loop through the succeeding rects, and evaluate the pair to see if it's
	 *          a better fit than what we've seen so far.
	 *
	 */
	if (centerXs.size() > 1) {
		std::vector<cv::Rect> createdRects;
		createdRects.reserve(centerXs.size());
		cv::Rect newRect;
		for(unsigned i = 0; i < centerXs.size(); i++) {
			newRect.height = heights[i];
			newRect.width = widths[i];
			newRect.x = centerXs[i] - (newRect.width / 2);
			newRect.y = centerYs[i] - (newRect.height / 2);
			createdRects.push_back(newRect);
		}

		int bestScore = 0;
		//Iterate through list of found contours.
		//ToDo
		for(unsigned int i=0; i < createdRects.size(); i++) {
			const cv::Rect & rectangle1 = createdRects[i];

			for(unsigned int j = i + 1; j < createdRects.size(); j++) {
				const cv::Rect & rectangle2 = createdRects[j];

				//Calculate a total score across all 6 measurements
				double scoreTotal = 0;
				scoreTotal += boundingRatioScore(rectangle1, rectangle2);
				scoreTotal += contourWidthScore(rectangle1, rectangle2);
				scoreTotal += topEdgeScore(rectangle1, rectangle2);
				scoreTotal += leftSpacingScore(rectangle1, rectangle2);
				scoreTotal += widthRatioScore(rectangle1, rectangle2);
				scoreTotal += heightRatioScore(rectangle1, rectangle2);

				if (scoreTotal > bestScore){
					bestScore = scoreTotal;
					boundingRect bounds(rectangle1, rectangle2);
					bestRectangle.x = bounds.left;
					bestRectangle.y = bounds.top;
					bestRectangle.width = bounds.right - bounds.left;
					bestRectangle.height = bounds.bottom - bounds.top;
				}
			}
		}
	}
#ifdef ENABLE_DEBUGGING_OUTPUT
	std::cout << "Size of rect is: "  << bestRectangle.height << " by " << bestRectangle.width << std::endl;
	std::cout << "Image Rect: " << imageRect << "Best Rectangle " << bestRectangle << std::endl;
#endif	// ENABLE_DEBUGGING_OUTPUTdasfj
}

void TapeTracker::visionExecuter()
{
	std::cerr << "Starting up vision execution" << std::endl;
	do {
		// Remember how big the source image was.
		//
		cv::Rect srcRect;
		srcRect.x = 0;
		srcRect.y = 0;
		srcRect.width = IMG_WIDTH * WIDTH_SCALING;
		srcRect.height = IMG_HEIGHT * HEIGHT_SCALING;

		//If we have at least 2 contours, we might have a target
		cv::Rect bestRectangle;

		processDriverStationData(bestRectangle);

#ifdef ENABLE_DEBUGGING_OUTPUT
		std::cerr << "Rectangles: \n"
				  << "  - Image:  " << srcRect << '\n'
				  << "  - Target: " << bestRectangle << std::endl;
#endif	// ENABLE_DEBUGGING_OUTPUT

		m_lock->lock();
		currentRect = bestRectangle;
		imageRect = srcRect;
		m_lock->unlock();
	} while(true);
}

void TapeTracker::getBoundingRects(cv::Rect& imageRect, cv::Rect& currentRect) const {
	m_lock->lock();
	imageRect = this->imageRect;
	currentRect = this->currentRect;
	m_lock->unlock();
}
