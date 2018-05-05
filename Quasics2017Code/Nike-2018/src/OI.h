#ifndef OI_H
#define OI_H

#include <WPILib.h>

/*
 * Joystic and driver station setup, and joystick interface
 */

class OI {
private:
	std::shared_ptr<Joystick> auxStick;		//Second controller, controls all but drive base
	std::shared_ptr<Joystick> driveStick;		//Primary controller, controls drive base

public:
	OI();		//Constructor for the joysticks and initializer for dashboard controls

	std::shared_ptr<Joystick> getDriveStick();		//Returns a pointer to the primary control stick
	std::shared_ptr<Joystick> getAuxStick();		//Returns a pointer to the secondary control stick
};

#endif
