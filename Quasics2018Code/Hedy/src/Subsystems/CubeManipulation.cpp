// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.




#include "CubeManipulation.h"
#include "../RobotMap.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

CubeManipulation::CubeManipulation() : frc::Subsystem("CubeManipulation") {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    limitSwitch = RobotMap::cubeManipulationLimitSwitch;
    leftShoulderMotor = RobotMap::cubeManipulationleftShoulderMotor;
    rightShoulderMotor = RobotMap::cubeManipulationrightShoulderMotor;
    shoulderMotors = RobotMap::cubeManipulationShoulderMotors;
    leftIntakeMotor = RobotMap::cubeManipulationleftIntakeMotor;
    rightIntakeMotor = RobotMap::cubeManipulationrightIntakeMotor;
    intakeMotors = RobotMap::cubeManipulationIntakeMotors;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
}

void CubeManipulation::InitDefaultCommand() {
    // Set the default command for a subsystem here.
    // SetDefaultCommand(new MySpecialCommand());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}
void CubeManipulation::SetShoulderPower(double percent){
	shoulderMotors->Set(percent);
}


void CubeManipulation::SetIntakePower(double percent){
	intakeMotors->Set(percent);
}

void CubeManipulation::Stop(){
	SetIntakePower(0);
	SetShoulderPower(0);
}


// Put methods for controlling this subsystem
// here. Call these from Commands.

