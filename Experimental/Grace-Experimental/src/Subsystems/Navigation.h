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
			return 0;
		}
	}
};

#endif /* SRC_SUBSYSTEMS_NAVIGATION_H_ */
