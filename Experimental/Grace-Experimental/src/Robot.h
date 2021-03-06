// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#ifndef _ROBOT_H
#define _ROBOT_H

#include "WPILib.h"
#include "Commands/Command.h"
#include "RobotMap.h"
#include "LiveWindow/LiveWindow.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "Commands/AutonomousCommand.h"
#include "Subsystems/DriveBase.h"
#include "Subsystems/Lighting.h"
#include "Subsystems/NavAlt.h"
#include "Subsystems/Navigation.h"
#include "Subsystems/Vision.h"


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "OI.h"

class Robot : public frc::TimedRobot {
public:
	static std::unique_ptr<OI> oi;

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	static std::shared_ptr<DriveBase> driveBase;
	static std::shared_ptr<Lighting> lighting;
	static std::shared_ptr<Vision> vision;
	static std::shared_ptr<Navigation> navigation;
	static std::shared_ptr<NavAlt> navAlt;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	void RobotInit() override;

	// Handling for "Disabled" mode
	void DisabledInit() override;
	void DisabledPeriodic() override;

	// Handling for "Autonomous" mode
	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	// Handling for "Teleoperated" mode
	void TeleopInit() override;
	void TeleopPeriodic() override;

	// Handling for "Testing" mode
	void TestInit() override;
	void TestPeriodic() override;

private:
	std::shared_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;
	//	frc::LiveWindow *lw = frc::LiveWindow::GetInstance();

	void StopAutoModeCommand();
};
#endif
