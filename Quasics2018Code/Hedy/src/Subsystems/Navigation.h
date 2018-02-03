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
	bool isReady();

	/**
	 * Resets the base value for the bearing, so that future queries will
	 * start from the current orientation.
	 *
	 * @see getBearing()
	 */
	void resetBearing();

	/**
	 * @return a value in the range of -180 to +180; if the sensor is
	 *         unavailable, a value of 0 will be returned.
	 * @see resetBearing()
	 */
	float getBearing();

	float getCompassHeading();

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
	float getAngle();

};

#endif /* SRC_SUBSYSTEMS_NAVIGATION_H_ */
