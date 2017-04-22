#include "Robot.h"

std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<GearSystem> Robot::gearSystem;
std::shared_ptr<ClimberSystem> Robot::climberSystem;
std::shared_ptr<ArduinoController> Robot::arduinoController;
std::unique_ptr<OI> Robot::oi;
std::shared_ptr<CommandGroup> Robot::teleopCommand;

void Robot::RobotInit() {
	RobotMap::init();
    driveBase.reset(new DriveBase);
    gearSystem.reset(new GearSystem);
    climberSystem.reset(new ClimberSystem);
    arduinoController.reset(new ArduinoController);
    teleopCommand.reset(new TeleopCommandGroup);

	oi.reset(new OI());
  }

void Robot::DisabledInit(){

}

void Robot::DisabledPeriodic() {
}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	teleopCommand->Start();
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
}

START_ROBOT_CLASS(Robot);

