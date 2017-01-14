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
#include "LiveWindow/LiveWindow.h"
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
std::shared_ptr<SpeedController> RobotMap::driveBaseLeftMotor;
std::shared_ptr<SpeedController> RobotMap::driveBaseRightMotor;
std::shared_ptr<RobotDrive> RobotMap::driveBaseRobotDrive21;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION

void RobotMap::init() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    LiveWindow *lw = LiveWindow::GetInstance();

    driveBaseLeftMotor.reset(new Talon(0));
    lw->AddActuator("Drive Base", "Left Motor", std::static_pointer_cast<Talon>(driveBaseLeftMotor));
    
    driveBaseRightMotor.reset(new Talon(1));
    lw->AddActuator("Drive Base", "Right Motor", std::static_pointer_cast<Talon>(driveBaseRightMotor));
    
    driveBaseRobotDrive21.reset(new RobotDrive(driveBaseLeftMotor, driveBaseRightMotor));
    
    driveBaseRobotDrive21->SetSafetyEnabled(true);
        driveBaseRobotDrive21->SetExpiration(0.1);
        driveBaseRobotDrive21->SetSensitivity(0.5);
        driveBaseRobotDrive21->SetMaxOutput(1.0);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}
