#ifndef _ROBOT_H
#define _ROBOT_H

//------------------------Library Includes---------------------------------------------------------
#include "WPILib.h"		//WPI Library (FRC Libraries)
//------------------------Command Includes---------------------------------------------------------
#include <Commands/CommandGroups/TeleopCommandGroup.h>		//Default Teleop Command Group
#include <Commands/Lights/AutomaticLighting.h>		//Automatic Lighting Command
#include <Commands/CommandGroups/Autonomous.h>		//Autonomous command group
//------------------------Subsystem Includes-------------------------------------------------------
#include "Subsystems/ClimberSystem.h"		//Climbing subsystem
#include "Subsystems/DriveBase.h"		//Drive Train subsystem
#include "Subsystems/GearSystem.h"		//Gear Delivery subsystem
#include "Subsystems/ArduinoController.h"		//Arduino COmmunications subsystem
//------------------------Misc. Includes-----------------------------------------------------------
#include "RobotMap.h"		//Map of all Objects on the robot
#include "OI.h"		//Map of the Operator interface (joysticks and the smart dashboard buttons)
//-------------------------------------------------------------------------------------------------

class Robot: public IterativeRobot {
public:
	//-------------------------Subsystem Pointers--------------------------------------------------
	static std::shared_ptr<CommandGroup> autonomousCommand;		//Autonomous command to run
	static std::shared_ptr<CommandGroup> teleopCommand;		//Teleop command group to start automatically
	static std::shared_ptr<Command>	lightingCommand;		//Automatic lighting command
	//-------------------------Command Pointers----------------------------------------------------
	static std::shared_ptr<DriveBase> driveBase;		//The drive base system
	static std::shared_ptr<GearSystem> gearSystem;		//the gear delivery system
	static std::shared_ptr<ClimberSystem> climberSystem;		//the climber system
	static std::shared_ptr<ArduinoController> arduinoController;		//Arduino Communications
	//-------------------------Other Pointers------------------------------------------------------
	static std::unique_ptr<OI> oi;		//The operator control interface
	//---------------------------------------------------------------------------------------------

	virtual void RobotInit();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestPeriodic();
};
#endif
