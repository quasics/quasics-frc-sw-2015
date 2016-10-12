// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "MoveArmForTime.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

MoveArmForTime::MoveArmForTime(double seconds, double power): Command() {
    m_seconds = seconds;
    m_power = power;
        // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::shooterArm.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

	stopTime = int(m_seconds * 50 + .5); //Convert Seconds to cycles
	timer = 0; //set cycle counter to 0
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void MoveArmForTime::Initialize() {
	Robot::shooterArm->DirectArmControll(m_power);//Set arm to specified power
}

// Called repeatedly when this Command is scheduled to run
void MoveArmForTime::Execute() {
	timer++; //increment cycle counter
}

// Make this return true when this Command no longer needs to run execute()
bool MoveArmForTime::IsFinished() {
    return timer >= stopTime; //Return true if you have reached or exceeded the target cycle count
}

// Called once after isFinished returns true
void MoveArmForTime::End() {
	Robot::shooterArm->StopArm(); //Stop arm motion when command has ended
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MoveArmForTime::Interrupted() {
	Robot::shooterArm->StopArm(); //Stop arm motion when command has been interrupted
}
