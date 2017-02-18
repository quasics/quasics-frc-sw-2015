#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H

#include <WPILib.h>
#include "../ConditionalFlags.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveBase: public Subsystem {
private:
	std::shared_ptr<SpeedController> leftFrontMotor;
	std::shared_ptr<SpeedController> leftRearMotor;
	std::shared_ptr<SpeedController> rightFrontMotor;
	std::shared_ptr<SpeedController> rightRearMotor;
	std::shared_ptr<Joystick>        driveStick;

#ifdef USE_FRC_DRIVE_CODE
	std::shared_ptr<RobotDrive>      robotDrive21;
#endif

public:
	DriveBase();
	void InitDefaultCommand();

	void tankDrive();

#ifndef USE_FRC_DRIVE_CODE
private:
	void setLeftPower(float value);
	void setRightPower(float value);
#endif
};

#endif
