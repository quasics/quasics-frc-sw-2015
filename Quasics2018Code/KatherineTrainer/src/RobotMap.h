// RobotBuilder Version: 2.0
#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "WPILib.h"

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
class RobotMap {
public:
	static std::shared_ptr<frc::SpeedController> driveBaseLeftFrontMotor;
	static std::shared_ptr<frc::SpeedController> driveBaseLeftRearMotor;
	static std::shared_ptr<frc::SpeedControllerGroup> driveBaseLeftMotors;
	static std::shared_ptr<frc::SpeedController> driveBaseRightFrontMotor;
	static std::shared_ptr<frc::SpeedController> driveBaseRightRearMotor;
	static std::shared_ptr<frc::SpeedControllerGroup> driveBaseRightMotors;

	static void init();
};
#endif
