/*
 * Navigation.h
 *
 *  Created on: Nov 4, 2017
 *      Author: Developer
 */

#ifndef SRC_SUBSYSTEMS_NAVIGATION_H_
#define SRC_SUBSYSTEMS_NAVIGATION_H_

#include <Commands/Subsystem.h>
#include "../ThirdParty/NavX/include/AHRS.h"
#include <iostream>

class Navigation: public frc::Subsystem {
////////////////////////////////////////////////////////////////////////////////
// Private data.
private:
	std::shared_ptr<AHRS> ahrs;

public:
	Navigation();

////////////////////////////////////////////////////////////////////////////////
// Primitives for use by commands.
public:
	/**
	 * @return true iff the sensor is available for use
	 */
	bool isReady() {
		return (ahrs.get() != nullptr)
				&& ahrs->IsConnected()
				&& !ahrs->IsCalibrating();
	}

	/**
	 * Resets the base value for the bearing, so that future queries will
	 * start from the current orientation.
	 *
	 * @see getBearing()
	 */
	void resetBearing() {
		if (ahrs.get() != nullptr) {
			ahrs->ZeroYaw();
		}
	}

	/**
	 * @return a value in the range of -180 to +180; if the sensor is
	 *         unavailable, a value of 0 will be returned.
	 * @see resetBearing()
	 */
	float getBearing() {
		if (isReady()) {
			return ahrs->GetYaw();
		} else {
			std::cerr << "*** Warning: NavX not ready ("  << __PRETTY_FUNCTION__ << "....\n";
			return 0;
		}
	}

	float getCompassHeading() {
		if (isReady()) {
			return ahrs->GetCompassHeading();
		}
		else {
			std::cerr << "*** Warning: NavX not ready ("  << __PRETTY_FUNCTION__ << "....\n";
			return 0;
		}
	}

	/**
	 * Returns the cumulative angle that the robot has passed through.
	 *
	 * Unlike "getBearing()", which is always in the range of -180 to
	 * +180, this is fully continuous.  In other words, if you turn in
	 * a full circle to the right twice, you should get 720; if you
     * then turned 3 full circles to the left, you'd get -360 (with the
     * first 2 circles canceling out the ones to the right, and then
     * -360 for the 3rd circle).
	 */
	float getAngle() {
		if (isReady()) {
			return ahrs->GetAngle();
		}
		else {
			std::cerr << "*** Warning: NavX not ready ("  << __PRETTY_FUNCTION__ << "....\n";
			return 0;
		}
	}

};

#endif /* SRC_SUBSYSTEMS_NAVIGATION_H_ */
