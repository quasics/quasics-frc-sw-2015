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
#include <WPILib.h>
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
std::shared_ptr<SpeedController> RobotMap::driveBaseleftFront;
std::shared_ptr<SpeedController> RobotMap::driveBaseleftRear;
std::shared_ptr<SpeedController> RobotMap::driveBaserightFront;
std::shared_ptr<SpeedController> RobotMap::driveBaserightRear;
std::shared_ptr<Encoder> RobotMap::driveBaseleftEncoder;
std::shared_ptr<Encoder> RobotMap::driveBaserightEncoder;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION

void RobotMap::init() {

	const int kLeftFrontPort = 0;		// 0
	const int kLeftRearPort = 1;		// 1
	const int kRightFrontPort = 2;		// 0
	const int kRightRearPort = 3;		// 1
    driveBaseleftFront.reset(new Talon(kLeftFrontPort));
    driveBaseleftFront->SetInverted(true);
    
    driveBaseleftRear.reset(new Talon(kLeftRearPort));
    driveBaseleftRear->SetInverted(true);
    
    driveBaserightFront.reset(new Talon(kRightFrontPort));
    driveBaserightRear.reset(new Talon(kRightRearPort));
    
    driveBaseleftEncoder.reset(new Encoder(0, 1, true));
    driveBaseleftEncoder->SetDistancePerPulse(InchesPerTick);
    driveBaseleftEncoder->SetPIDSourceType(PIDSourceType::kRate);

    driveBaserightEncoder.reset(new Encoder(2, 3, false));
    driveBaserightEncoder->SetDistancePerPulse(InchesPerTick);
    driveBaserightEncoder->SetPIDSourceType(PIDSourceType::kRate);
}
