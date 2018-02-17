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

#include "PortMappings.h"

#include <Drive/DifferentialDrive.h>
#include <Jaguar.h>
#include <SpeedController.h>
#include <SpeedControllerGroup.h>

#include <memory>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseFrontRight;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseBackRight;
std::shared_ptr<frc::SpeedControllerGroup> RobotMap::driveBaseRightMotors;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseFrontLeft;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseBackLeft;
std::shared_ptr<frc::SpeedControllerGroup> RobotMap::driveBaseLeftMotors;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION

template<typename Widget>
void SetNameAndSubsystem(Widget& w, const char* subsystemName, const char* motorName) {
	w.SetName(subsystemName, motorName);
}
template<typename Motor>
std::shared_ptr<Motor> createMotor(int port, const char* subsystemName, const char* motorName) {
	std::unique_ptr<Motor> motor(new Motor(port));
	SetNameAndSubsystem(*motor, subsystemName, motorName);
	return std::shared_ptr<Motor>(motor.release());
}

void RobotMap::init() {
#ifdef USE_ROBOT_BUILDER_AUTOGEN_CODE
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    frc::LiveWindow *lw = frc::LiveWindow::GetInstance();

    driveBaseFrontRight.reset(new frc::Jaguar(1));
    lw->AddActuator("DriveBase", "Front Right", std::static_pointer_cast<frc::Jaguar>(driveBaseFrontRight));
    
    driveBaseBackRight.reset(new frc::Jaguar(0));
    lw->AddActuator("DriveBase", "Back Right", std::static_pointer_cast<frc::Jaguar>(driveBaseBackRight));
    
    driveBaseRightMotors = std::make_shared<frc::SpeedControllerGroup>(*driveBaseFrontRight, *driveBaseBackRight  );
    lw->AddActuator("DriveBase", "Right Motors", driveBaseRightMotors);
    
    driveBaseFrontLeft.reset(new frc::Jaguar(2));
    lw->AddActuator("DriveBase", "Front Left", std::static_pointer_cast<frc::Jaguar>(driveBaseFrontLeft));
    
    driveBaseBackLeft.reset(new frc::Jaguar(3));
    lw->AddActuator("DriveBase", "Back Left", std::static_pointer_cast<frc::Jaguar>(driveBaseBackLeft));
    
    driveBaseLeftMotors = std::make_shared<frc::SpeedControllerGroup>(*driveBaseFrontLeft, *driveBaseBackLeft  );
    lw->AddActuator("DriveBase", "Left Motors", driveBaseLeftMotors);
    


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
#else
    driveBaseFrontRight = createMotor<frc::Jaguar>(FRONT_RIGHT_MOTOR_PORT, "Drive Base", "frontRight");
    driveBaseBackRight = createMotor<frc::Jaguar>(BACK_RIGHT_MOTOR_PORT, "Drive Base", "backRight");
    driveBaseFrontLeft = createMotor<frc::Jaguar>(FRONT_LEFT_MOTOR_PORT, "Drive Base", "frontLeft");
    driveBaseBackLeft = createMotor<frc::Jaguar>(BACK_LEFT_MOTOR_PORT, "Drive Base", "backLeft");

    driveBaseLeftMotors = std::make_shared<frc::SpeedControllerGroup>(*driveBaseFrontLeft, *driveBaseBackLeft  );
    SetNameAndSubsystem(*driveBaseLeftMotors, "DriveBase", "Left Motors");
    driveBaseRightMotors = std::make_shared<frc::SpeedControllerGroup>(*driveBaseFrontLeft, *driveBaseBackLeft  );
    SetNameAndSubsystem(*driveBaseLeftMotors, "DriveBase", "Left Motors");
#endif
}
