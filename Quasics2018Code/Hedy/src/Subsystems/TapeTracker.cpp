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

#define WIDTH_SCALING		.7
#define HEIGHT_SCALING		.7

#define ENABLE_DEBUGGING_OUTPUT


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



/*
 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
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

TapeTracker::TapeTracker() : frc::Subsystem("TapeTracker") {
		//table = NetworkTable::GetTable("datatable");
	cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
		camera.SetResolution(IMG_WIDTH, IMG_HEIGHT);
		m_lock = new std::mutex;

		visionTrackingTask = new frc::VisionRunner<grip::Vision>(
				camera, new grip::Vision(),
				[&](grip::Vision& pipeline)
				{
					// Remember how big the source image was.
					//
					cv::Rect srcRect;
					srcRect.x = 0;
					srcRect.y = 0;
					srcRect.width = IMG_WIDTH * WIDTH_SCALING;
					srcRect.height = IMG_HEIGHT * HEIGHT_SCALING;

					//If we have at least 2 contours, we might have a target
					cv::Rect bestRectangle;
					//const auto & filterContoursOutput = *pipeline.GetFilterContoursOutput();

					//TODO: Read data from the network table;

						std::vector<double> centerXs = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
						std::vector<double> centerYs = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
						std::vector<double> widths = table->GetNumberArray("width", llvm::ArrayRef<double>());
						std::vector<double> heights = table->GetNumberArray("height", llvm::ArrayRef<double>());

					// Note that we'll assume all arrays are of the same size (and nothing changes while we
					// were grabbing them).
					if (centerXs.empty()) {
					// No contours were apparently seen
						std::cout << "Don't got rects " << std::endl;
					}
					else{
						std::cout << "Got rects: " << std::endl;

					}





/*#ifdef ENABLE_DEBUGGING_OUTPUT
					std::cerr << "# of contours: " << filterContoursOutput.size() << std::endl;
#endif	// ENABLE_DEBUGGING_OUTPUT

					if (filterContoursOutput.size() > 1)
					{
						int bestScore = 0;
						//Iterate through list of found contours.
						//ToDo
						std::vector<double> centerX = table -> GetNumberArray("centerX", llvm::ArrayRef<double>());
						std::vector<double> centerY = table -> GetNumberArray("centerY", llvm::ArrayRef<double>());
						std::vector<double> height = table -> GetNumberArray("height", llvm::ArrayRef<double>());
						std::vector<double> area = table -> GetNumberArray("area", llvm::ArrayRef<double>());
						std::vector<double> width = table -> GetNumberArray("width", llvm::ArrayRef<double>());
					//	std::cout << centerX << centerY << area << height << width;

						for(unsigned int i=0; i < filterContoursOutput.size(); i++)
						{
							const std::vector<cv::Point> & countourPoints1 = filterContoursOutput[i];
							const cv::Rect rectangle1 = cv::boundingRect(cv::Mat(countourPoints1));

							for(unsigned int j = i + 1; j < filterContoursOutput.size(); j++) {
								const std::vector<cv::Point> & countourPoints2 = filterContoursOutput[j];
								const cv::Rect rectangle2 = cv::boundingRect(cv::Mat(countourPoints2));

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
					*/


					m_lock->lock();
					currentRect = bestRectangle;
					imageRect = srcRect;
					m_lock->unlock();
				});
		m_visionThread = new std::thread(&TapeTracker::visionExecuter, this);

}


void TapeTracker::visionExecuter()
{
	std::cerr << "Starting up vision execution" << std::endl;
	visionTrackingTask->RunForever();
}

void TapeTracker::getBoundingRects(cv::Rect& imageRect, cv::Rect& currentRect) const {
	m_lock->lock();
	imageRect = this->imageRect;
	currentRect = this->currentRect;
	m_lock->unlock();
}

