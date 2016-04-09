// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "StubCommand2.h"

#include <Commands/Subsystem.h>
#include "../Robot.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

StubCommand2::StubCommand2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    Requires(Robot::driveSystem.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    isFinished = false;
    autoStage = 0;
}

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void StubCommand2::Initialize() {
    isFinished = false;
    autoStage = 0;
    Robot::driveSystem->ResetYaw();
    Robot::driveSystem->ResetEncoders();

}

// Called repeatedly when this Command is scheduled to run
void StubCommand2::Execute() {
    switch (autoStage) {
        case 0:
            Robot::driveSystem->ResetEncoders();
            Robot::driveSystem->ResetYaw();
            autoStage = 1;
            break;
        case 1:
            //Motor Stuff
            if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
                Robot::driveSystem->MoveLeft(50);
                Robot::driveSystem->MoveRight(45);
            }
            else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
                Robot::driveSystem->MoveLeft(45);
                Robot::driveSystem->MoveRight(50);
            }
            else {
                Robot::driveSystem->MoveLeft(50);
                Robot::driveSystem->MoveRight(50);
            }

            if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
                    >= 1.95) {
                Robot::driveSystem->ResetEncoders();
                Robot::driveSystem->ResetYaw();
                autoStage = 4;
                //reset Sensors
            }
            break;
        case 2:
            Robot::driveSystem->MoveLeft(50);
            Robot::driveSystem->MoveRight(-50);

            if (Robot::driveSystem->GetContinuousYaw() >= -30) {
                Robot::driveSystem->ResetEncoders();
                Robot::driveSystem->ResetYaw();
                autoStage = 5;
            }
            break;
        case 3:
            //Motor Stuff
            if (Robot::driveSystem->GetContinuousYaw() > 2.5) {
                Robot::driveSystem->MoveLeft(50);
                Robot::driveSystem->MoveRight(45);
            }
            else if (Robot::driveSystem->GetContinuousYaw() < -2.5) {
                Robot::driveSystem->MoveLeft(45);
                Robot::driveSystem->MoveRight(50);
            }
            else {
                Robot::driveSystem->MoveLeft(50);
                Robot::driveSystem->MoveRight(50);
            }

            if (Robot::driveSystem->GetEncoderDistance(DriveSystem::kLeft)
                    >= 2.53) {
                Robot::driveSystem->ResetEncoders();
                Robot::driveSystem->ResetYaw();
                isFinished = true;
                //reset Sensors
            }
            break;
    }

}
// Make this return true when this Command no longer needs to run execute()
bool StubCommand2::IsFinished() {
    return isFinished;
}

// Called once after isFinished returns true
void StubCommand2::End() {
    Robot::driveSystem->StopEverything();

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StubCommand2::Interrupted() {
    Robot::driveSystem->StopEverything();

}
