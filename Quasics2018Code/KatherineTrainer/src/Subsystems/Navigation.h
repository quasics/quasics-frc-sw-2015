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

	void resetBearing() {
		if (ahrs.get() != nullptr) {
			ahrs->ZeroYaw();
		}
	}

	float getBearing() {
		if (isReady()) {
			return ahrs->GetYaw();
		} else {
			return 0;
		}
	}

	/// Returns a value in the range of -180 to +180, with 0 meaning "North".
	float getCompassHeadingRelativeToNorth() {
		float heading = getCompassHeading();
		if (heading > 180) {
			heading -= 360;
		}
		return heading;
	}

	/// Returns a compass heading in the range 0 to 360, with 0 meaning "North".
	float getCompassHeading() {
		if (isReady()) {
			float result = ahrs->GetCompassHeading();
			std::cout << "Current heading: " << result << std::endl;
			return result;
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

#endif /* SRC_SUBSYSTEMS_NAVIGATION_H_ */
