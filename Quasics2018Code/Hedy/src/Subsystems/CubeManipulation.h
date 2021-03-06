// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef CUBEMANIPULATION_H
#define CUBEMANIPULATION_H

#include <WPILib.h>

/**
 *
 *
 * @author ExampleAuthor
 */
class CubeManipulation: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::SpeedController> leftShoulderMotor;
	std::shared_ptr<frc::SpeedController> rightShoulderMotor;
	std::shared_ptr<frc::SpeedControllerGroup> shoulderMotors;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	CubeManipulation();
	void InitDefaultCommand() override;
	void SetShoulderPower(double percent = 0);
	void Stop();

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
};

#endif
