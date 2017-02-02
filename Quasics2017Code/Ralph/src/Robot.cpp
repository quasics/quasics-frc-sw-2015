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
#include "Commands/MoveInSquare.h"

std::shared_ptr<DriveTrain> Robot::driveTrain;
std::shared_ptr<Navigation> Robot::gyro;
std::shared_ptr<Intake> Robot::intake;
std::shared_ptr<Outtake> Robot::outtake;
std::shared_ptr<Gear> Robot::gear;
std::unique_ptr<OI> Robot::oi;



void Robot::RobotInit() {
	RobotMap::init();
    driveTrain.reset(new DriveTrain());
    gyro.reset(new Navigation());
    outtake.reset(new Outtake());
    gear.reset(new Gear());
    intake.reset(new Intake());
	oi.reset(new OI());
	autonomousCommand.reset(new MoveForTime(1, 1));
	autoCommand.reset(new MoveInSquare());

//Camera Commands
#ifdef Use_Camera
	CameraServer::GetInstance()->SetQuality(50);
	CameraServer::GetInstance()->StartAutomaticCapture("cam0");
#endif
  }

/**
 * This function is called when the disabled button is hit.
 * You can use it to reset subsystems before shutting down.
 */


void Robot::DisabledInit(){

}


void Robot::DisabledPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	if (autonomousCommand.get() != nullptr)
		autonomousCommand->Start();
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	if (autonomousCommand.get() != nullptr)
		autonomousCommand->Cancel();
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	lw->Run();
}

START_ROBOT_CLASS(Robot);

