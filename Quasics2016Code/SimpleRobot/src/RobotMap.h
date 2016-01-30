// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


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
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	static std::shared_ptr<SpeedController> driveSystemLeftFront;
	static std::shared_ptr<SpeedController> driveSystemLeftRear;
	static std::shared_ptr<SpeedController> driveSystemRightFront;
	static std::shared_ptr<SpeedController> driveSystemRightRear;
	static std::shared_ptr<DigitalInput> antonioShooterBasketMax;
	static std::shared_ptr<DigitalInput> antonioShooterBasketMin;
	static std::shared_ptr<Servo> antonioShooterBasketServo;
	static std::shared_ptr<SpeedController> antonioShooterLeftArmMotor;
	static std::shared_ptr<SpeedController> antonioShooterRightArmMotor;
	static std::shared_ptr<Encoder> antonioShooterLeftArmEncoder;
	static std::shared_ptr<Encoder> antonioShooterRightArmEncoder;
	static std::shared_ptr<SpeedController> antonioShooterLeftShooterMotor;
	static std::shared_ptr<SpeedController> antonioShooterRightShooterMotor;
	static std::shared_ptr<Encoder> antonioShooterLeftShooterEncoder;
	static std::shared_ptr<Encoder> antonioShooterRightShooterencoder;
	static std::shared_ptr<Relay> mrPlowIntakeMotor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	static void init();
};
#endif
