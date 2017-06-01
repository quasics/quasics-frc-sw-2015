#include "Robot.h"

std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<GearSystem> Robot::gearSystem;
std::shared_ptr<ClimberSystem> Robot::climberSystem;
std::shared_ptr<ArduinoController> Robot::arduinoController;
std::unique_ptr<OI> Robot::oi;
std::shared_ptr<CommandGroup> Robot::teleopCommand;
std::shared_ptr<Command> Robot::lightingCommand;
std::shared_ptr<CommandGroup> Robot::autonomousCommand;

void Robot::RobotInit() {
	CameraServer::GetInstance()->StartAutomaticCapture(0);
	RobotMap::init();
	driveBase.reset(new DriveBase);
	gearSystem.reset(new GearSystem);
	climberSystem.reset(new ClimberSystem);
	arduinoController.reset(new ArduinoController);
	teleopCommand.reset(new TeleopCommandGroup);
	lightingCommand.reset(new AutomaticLighting);
	autonomousCommand.reset(new Autonomous);

	oi.reset(new OI());
}

void Robot::DisabledInit() {
	lightingCommand->Start();

}

void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	if (lightingCommand != NULL) {
		lightingCommand->Start();
	}
	if (autonomousCommand != NULL) {
		autonomousCommand->Start();
	}
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();

}

void Robot::TeleopInit() {
	teleopCommand->Start();
	lightingCommand->Start();
	if (lightingCommand != NULL) {
		lightingCommand->Start();
	}
	if (teleopCommand != NULL) {
		teleopCommand->Start();
	}
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	Scheduler::GetInstance()->Run();
}

START_ROBOT_CLASS(Robot);

