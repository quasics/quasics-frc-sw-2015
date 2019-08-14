// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef NEWELEVATOR_H
#define NEWELEVATOR_H
#include "frc/commands/Subsystem.h"
#include "frc/WPILib.h"

/**
 *
 *
 * @author ExampleAuthor
 */
class NewElevator: public frc::Subsystem {
private:
	// It's desirable that everything possible is private except
	// for methods that implement subsystem capabilities
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	std::shared_ptr<frc::SpeedController> leftElevatorMotor;
	std::shared_ptr<frc::SpeedController> rightElevatorMotor;
	std::shared_ptr<frc::SpeedControllerGroup> elevatorMotors;
	std::shared_ptr<frc::DigitalInput> highStop;
	std::shared_ptr<frc::DigitalInput> lowStop;
	std::shared_ptr<frc::DigitalInput> lowScoring;
	std::shared_ptr<frc::DigitalInput> middleScoring;
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
public:
	NewElevator();
	void InitDefaultCommand() override;
	void Periodic() override;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
	bool atBottom();
	bool atTop();
	bool atLow();
	bool atMedium();

	void move(double speed);
	void stop();
};

#endif
