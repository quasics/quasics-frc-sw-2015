#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <WPILib.h>
#include <AHRS.h>


/**
 * @author mjhealy@alum.rit.edu, based on a skeleton from RobotBuilder.
 */
class Navigation: public Subsystem {
////////////////////////////////////////////////////////////////////////////////
// Private data.
private:
	std::shared_ptr<AHRS> ahrs;

////////////////////////////////////////////////////////////////////////////////
// Housekeeping and core infrastructure.
public:
	Navigation();
	void InitDefaultCommand();

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

#endif
