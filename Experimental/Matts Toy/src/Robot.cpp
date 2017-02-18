#include "Robot.h"

#include "RobotMap.h"
#include "Commands/SimpleAutoCommand.h"

std::shared_ptr<DriveBase> Robot::driveBase;
std::shared_ptr<Navigation> Robot::navigation;
std::shared_ptr<Lighting> Robot::lighting;
std::unique_ptr<OI> Robot::oi;

void Robot::RobotInit() {
	// Step 1: initialize the RobotMap (instantiating devices).
	RobotMap::init();

	// Step 2: set up the subsystems (which use the devices from the map).
    driveBase.reset(new DriveBase());
    navigation.reset(new Navigation());
    lighting.reset(new Lighting());

    // Step 3: set up the operator interface.
	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi.reset(new OI());

	// Step 4: instantiate the command used for the autonomous period
	autonomousCommand.reset(new SimpleAutoCommand());
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
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// these lines or comment it out.
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
