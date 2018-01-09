#include "RobotMap.h"
#include "PortMappings.h"
#include "LiveWindow/LiveWindow.h"
#include <memory>

std::shared_ptr<SpeedController> RobotMap::driveBasefrontRight;
std::shared_ptr<SpeedController> RobotMap::driveBasebackRight;
std::shared_ptr<SpeedController> RobotMap::driveBasefrontLeft;
std::shared_ptr<SpeedController> RobotMap::driveBasebackLeft;

std::shared_ptr<SpeedController> RobotMap::createMotor(int port, const char* subsystemName, const char* motorName) {
	std::unique_ptr<Jaguar> motor(new Jaguar(port));
	motor->SetName(subsystemName, motorName);
	return std::shared_ptr<SpeedController>(motor.release());
}

void RobotMap::init() {
    driveBasefrontRight = createMotor(FRONT_RIGHT_MOTOR_PORT, "Drive Base", "frontRight");
    driveBasebackRight = createMotor(BACK_RIGHT_MOTOR_PORT, "Drive Base", "backRight");
    driveBasefrontLeft = createMotor(FRONT_LEFT_MOTOR_PORT, "Drive Base", "frontLeft");
    driveBasebackLeft = createMotor(BACK_LEFT_MOTOR_PORT, "Drive Base", "backLeft");
}
