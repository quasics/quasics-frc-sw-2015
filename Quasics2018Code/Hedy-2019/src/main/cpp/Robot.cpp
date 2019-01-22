// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>		// 	If needed, access via "frc::LiveWindow::GetInstance()"
#include <iostream>

#include "RobotMap.h"
#include "Commands/TeleOp/Teleop.h"
#include "Commands/AutoModeScoringCommand.h"
#include "Commands/DummyCommand.h"
#include "Commands/AutoCrossTheLineFromSideCommand.h"
#include "Commands/AutoModeCrossTheLineCommand.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION


std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<Navigation> Robot::navigation;
std::shared_ptr<CubeManipulation> Robot::cubeManipulation;
std::shared_ptr<CubeIntake> Robot::cubeIntake;
std::shared_ptr<Lighting> Robot::lighting;
std::shared_ptr<GyroADXRS> Robot::gyroADXRS;
std::unique_ptr<KatTrainerOI> Robot::oi;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION

Robot* Robot::theOneRobot = nullptr;


std::shared_ptr<frc::CommandGroup> Robot::teleopCommand;


// ENABLE this define, if we want to actually take some action when in Auto mode.
// DISABLE it, if we want to do nothing when in Auto mode.
//#define ENABLE_AUTO_MODE_OPERATION

Robot::CameraWrapper::CameraWrapper()
	: camera(CameraServer::GetInstance()->StartAutomaticCapture())
{
}

void Robot::RobotInit() {
	RobotMap::init();

	//////////////////////////////////////////////////////////////////////
	// Subsystem creation

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS


    driveBase.reset(new DriveBase());
    navigation.reset(new Navigation());
    cubeManipulation.reset(new CubeManipulation());
    cubeIntake.reset(new CubeIntake());
    lighting.reset(new Lighting());
    gyroADXRS.reset(new GyroADXRS());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    std::cout<< "Degrees" << Robot::navigation->getAngle() << std::endl;

    leftPlacementCommand = new DummyCommand;
    rightPlacementCommand = new DummyCommand;
    middlePlacementCommand = new DummyCommand;



    //////////////////////////////////////////////////////////////////////
    // OI (Operator Interface) creation and population

	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi.reset(new KatTrainerOI());

	// Add commands to Autonomous Sendable Chooser
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS


	chooser.AddDefault("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
	chooser.AddObject("Left Position", leftPlacementCommand);
	chooser.AddObject("Middle Position", middlePlacementCommand);
	chooser.AddObject("Right Position", rightPlacementCommand);
	frc::SmartDashboard::PutData("Auto Modes", &chooser);

	//-------------------SET Default Commands---------------------------------//

	autonomousCommand.reset(new AutoModeCrossTheLineCommand);
	teleopCommand.reset(new Teleop);


    //////////////////////////////////////////////////////////////////////
    // Misc. set-up

#ifdef CONFIGURE_CAMERA
	cameraWrapper.reset(new CameraWrapper);
	cameraWrapper->camera.SetResolution(320, 240);
#endif
}

/**
 * This function is called when the disabled button is hit.
 * You can use it to reset subsystems before shutting down.
 */
void Robot::DisabledInit(){

}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	const std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	determineStartingPosition();
	if (autonomousCommand != nullptr) {
		autonomousCommand->Start();
	}
}

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();

}

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
	if (autonomousCommand != nullptr) {
		autonomousCommand->Cancel();

	}

	if (teleopCommand != NULL) {		//If there is a defined teleop comman group
		teleopCommand->Start();		//Start the teleop command group
	}

}

void Robot::TeleopPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

Robot::RobotStartingPosition Robot::getStartingPosition() {
	// TODO: Implement code for figuring out what side we're on.
	if (theOneRobot == nullptr) {
		return eStartingInMiddle;
	} else {
		return theOneRobot->determineStartingPosition();
	}
}

Robot::RobotStartingPosition Robot::determineStartingPosition() {
	frc::Command* chosenValue = chooser.GetSelected();
	if (chosenValue == leftPlacementCommand) {
		return eStartingOnLeft;
	}
	else if (chosenValue == rightPlacementCommand) {
		return eStartingOnRight;
	}
	else if (chosenValue == middlePlacementCommand) {
		return eStartingInMiddle;

	} else {
		return eStartingInMiddle;

	}
}

// START_ROBOT_CLASS(Robot);

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
 
