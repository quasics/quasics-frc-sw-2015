// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef DRIVE_BASE_H
#define DRIVE_BASE_H
#include "Commands/Subsystem.h"
#include "WPILib.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class DriveBase: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::SpeedController> frontRight;
	std::shared_ptr<frc::SpeedController> backRight;
	std::shared_ptr<frc::SpeedControllerGroup> rightMotors;
	std::shared_ptr<frc::SpeedController> frontLeft;
	std::shared_ptr<frc::SpeedController> backLeft;
	std::shared_ptr<frc::SpeedControllerGroup> leftMotors;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	DriveBase();
	void InitDefaultCommand() override;
	void Periodic() override;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

	void SetPower(double left, double right);
	void Stop() { SetPower(0, 0); }
};

#endif
