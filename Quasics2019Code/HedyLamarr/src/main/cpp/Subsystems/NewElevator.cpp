// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "Subsystems/NewElevator.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

NewElevator::NewElevator() : frc::Subsystem("NewElevator") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    leftElevatorMotor.reset(new frc::Spark(5));
    rightElevatorMotor.reset(new frc::Spark(6));
    elevatorMotors = std::make_shared<frc::SpeedControllerGroup>(*leftElevatorMotor, *rightElevatorMotor  );
    highStop.reset(new frc::DigitalInput(3));
    lowStop.reset(new frc::DigitalInput(0));
    lowScoring.reset(new frc::DigitalInput(1));
    middleScoring.reset(new frc::DigitalInput(2));
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void NewElevator::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}

void NewElevator::Periodic() {
    // Put code here to be run every loop

}

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


// Put methods for controlling this subsystem
// here. Call these from Commands.
bool NewElevator::atTop() {
    return highStop->Get();
}

bool NewElevator::atBottom() {
    return lowStop->Get();
}

bool NewElevator::atLow() {
    return lowScoring->Get();
}

bool NewElevator::atMedium() {
    return middleScoring->Get();
}

void NewElevator::move(double speed){
    elevatorMotors->Set(speed);
}