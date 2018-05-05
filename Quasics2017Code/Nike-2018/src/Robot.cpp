//---------------------Header Include--------------------------------------------------------------
#include "Robot.h"

//-------------------------------------------------------------------------------------------------

//---------------------Subsystem Pointers----------------------------------------------------------
std::unique_ptr<OI> Robot::oi;		//Operator interface (i.e. the joysticks)
std::shared_ptr<DriveBase> Robot::driveBase;		//What it says on the tin
std::shared_ptr<GearSystem> Robot::gearSystem;		//Gear door and kicker
std::shared_ptr<ClimberSystem> Robot::climberSystem;		//Climber motor
std::shared_ptr<ArduinoController> Robot::arduinoController;//Arduino serial interface

//---------------------Command Pointers------------------------------------------------------------
std::shared_ptr<CommandGroup> Robot::teleopCommand;	//Command launched at the start of teleop
std::shared_ptr<Command> Robot::lightingCommand;//Lighting profile to use from startup
std::shared_ptr<CommandGroup> Robot::autonomousCommand;	//Command to run in autonomopus mode

//-------------------------------------------------------------------------------------------------

void Robot::RobotInit() {		//Runs on startup
	//---------------------Start Objects---------------------------------------------------------------
	CameraServer::GetInstance()->StartAutomaticCapture(0);//Start camera stream from usb camera 0 (default)
	RobotMap::init();//Initialize the pointers for all base objects on the bot
	oi.reset(new OI());
	//---------------------Start Subsystems------------------------------------------------------------
	driveBase.reset(new DriveBase);		//Initialize the drive base subsystem
	gearSystem.reset(new GearSystem);		//Initialize the gear subsystem
	climberSystem.reset(new ClimberSystem);	//Initialize the climber subsystem
	arduinoController.reset(new ArduinoController);	//Initialize the arduino interface subsystem
	//-------------------------------------------------------------------------------------------------

	//---------------------Set deault Commands---------------------------------------------------------
	teleopCommand.reset(new TeleopCommandGroup);	//Set default teleop command
	lightingCommand.reset(new AutomaticLighting);//Set decault lighting command
	autonomousCommand.reset(new Autonomous);		//Set autonomous command
	//-------------------------------------------------------------------------------------------------

	lightingCommand->Start();		//Start the default lighting command
}

void Robot::DisabledInit() {
	if (lightingCommand != NULL) {		//If there is a lighting command
		lightingCommand->Start();		//Start the automatic lights
	}

}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();		//Run the Chosen commands
}

void Robot::AutonomousInit() {
	if (lightingCommand != NULL) {		//If there is a lighting command
		lightingCommand->Start();		//Start the automatic lights
	}
	if (autonomousCommand != NULL) {		//If there is an autonomous command
		autonomousCommand->Start();		//Start the autonomous command
	}
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();		//Run the chosen commands

}

void Robot::TeleopInit() {
	if (lightingCommand != NULL) {		//If there is a defined lighting command
		lightingCommand->Start();		//Start the defined lighting command
	}
	if (teleopCommand != NULL) {		//If there is a defined teleop comman group
		teleopCommand->Start();		//Start the teleop command group
	}
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();		//Run the chosen commands
}

void Robot::TestPeriodic() {
	Scheduler::GetInstance()->Run();		//Run the chosen commands
}

START_ROBOT_CLASS(Robot);

