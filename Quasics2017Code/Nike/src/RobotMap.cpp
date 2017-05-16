// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "RobotMap.h"
#include "WPILib.h"
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
std::shared_ptr<SpeedController> RobotMap::driveBaseleftFront;
std::shared_ptr<SpeedController> RobotMap::driveBaseleftRear;
std::shared_ptr<SpeedController> RobotMap::driveBaserightFront;
std::shared_ptr<SpeedController> RobotMap::driveBaserightRear;
std::shared_ptr<Encoder> RobotMap::driveBaseleftEncoder;
std::shared_ptr<Encoder> RobotMap::driveBaserightEncoder;
std::shared_ptr<Servo> RobotMap::gearSystemgearDoorServo;
std::shared_ptr<Servo> RobotMap::gearSystemgearKickerServo;
std::shared_ptr<SpeedController> RobotMap::climberSystemClimberMotor;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION

void RobotMap::init() {
    driveBaseleftFront.reset(new Talon(0));
    driveBaseleftFront->SetInverted(true);
    
    driveBaseleftRear.reset(new Talon(1));
    driveBaseleftRear->SetInverted(true);
    
    driveBaserightFront.reset(new Talon(2));
    
    driveBaserightRear.reset(new Talon(3));
    
    driveBaseleftEncoder.reset(new Encoder(0, 1, true));
    driveBaseleftEncoder->SetDistancePerPulse(InchesPerTick);
    driveBaseleftEncoder->SetPIDSourceType(PIDSourceType::kRate);

    driveBaserightEncoder.reset(new Encoder(2, 3, false));
    driveBaserightEncoder->SetDistancePerPulse(InchesPerTick);
    driveBaserightEncoder->SetPIDSourceType(PIDSourceType::kRate);

    gearSystemgearDoorServo.reset(new Servo(5));
    
    gearSystemgearKickerServo.reset(new Servo(9));

    climberSystemClimberMotor.reset(new Spark(8));
    climberSystemClimberMotor->SetInverted(true);
}
