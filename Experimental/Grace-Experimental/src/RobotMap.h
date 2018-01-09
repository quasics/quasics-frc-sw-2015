#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include <WPILib.h>

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
class RobotMap {
private:
	static std::shared_ptr<SpeedController> createMotor(int port, const char* subsystemName, const char* motorName);
public:
	static std::shared_ptr<SpeedController> driveBasefrontRight;
	static std::shared_ptr<SpeedController> driveBasebackRight;
	static std::shared_ptr<SpeedController> driveBasefrontLeft;
	static std::shared_ptr<SpeedController> driveBasebackLeft;

	static void init();
};
#endif
