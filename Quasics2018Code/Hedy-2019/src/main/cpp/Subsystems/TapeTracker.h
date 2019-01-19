/*
 * This file declares a simple vision tracking subsystem, which will
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

#ifndef TapeTracker_H
#define TapeTracker_H

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <opencv2/core.hpp>

class TapeTracker : public frc::Subsystem {
private:
	// It's desirable that everything possible be private except
	// for methods that implement subsystem capabilities
	std::unique_ptr<cs::UsbCamera> usbCamera;
	std::thread* m_visionThread = nullptr;
	mutable std::mutex* m_lock = nullptr;
	cv::Rect currentRect;
	cv::Rect imageRect;
	std::shared_ptr<nt::NetworkTable> table;

private:
	void processDriverStationData(cv::Rect& bestRectangle);

public:
	// Must be public, so that it can be invoked on a dedicated thread.
	void visionExecuter();

public:
	TapeTracker();

	cv::Rect getCurrentRect() const;
	cv::Rect getImageRect() const;

	void getBoundingRects(cv::Rect& imageRect, cv::Rect& currentRect) const;
};

#endif  // TapeTracker_H
