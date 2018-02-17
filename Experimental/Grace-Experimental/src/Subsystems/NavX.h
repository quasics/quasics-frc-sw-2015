/*
 * Navigation.h
 *
 *  Created on: Nov 4, 2017
 *      Author: Developer
 */

#ifndef SRC_SUBSYSTEMS_NAVX_H_
#define SRC_SUBSYSTEMS_NAVX_H_

#include <Commands/Subsystem.h>
#include "../ThirdParty/NavX/include/AHRS.h"

class NavX: public frc::Subsystem {
////////////////////////////////////////////////////////////////////////////////
// Private data.
private:
	std::shared_ptr<AHRS> ahrs;

public:
	NavX();

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

	float getCompassHeading() {
		if (isReady()) {
			return ahrs->GetCompassHeading();
		}
		else {
			return 0;
		}
	}

	float getAngle() {
		if (isReady()) {
			return ahrs->GetAngle();
		}
		else {
			return 0;
		}
	}


};

#endif /* SRC_SUBSYSTEMS_NAVX_H_ */
