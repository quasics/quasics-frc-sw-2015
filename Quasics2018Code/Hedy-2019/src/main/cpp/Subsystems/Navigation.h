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

#endif /* SRC_SUBSYSTEMS_NAVIGATION_H_ */
