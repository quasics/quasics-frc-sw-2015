#ifndef TapeTracker_H
#define TapeTracker_H

#include <Commands/Subsystem.h>
#include <WPILib.h>
#include <opencv2/core.hpp>
#include "Vision.h"


class TapeTracker : public frc::Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	frc::VisionRunner<grip::Vision> * visionTrackingTask;
	cv::Rect currentRect;
	cv::Rect imageRect;
	mutable std::mutex* m_lock;
	std::thread* m_visionThread;


public:
	TapeTracker();
	void visionExecuter();

	cv::Rect getCurrentRect() const;
		cv::Rect getImageRect() const;

		void getBoundingRects(cv::Rect& imageRect, cv::Rect& currentRect) const;
};

#endif  // TapeTracker_H
