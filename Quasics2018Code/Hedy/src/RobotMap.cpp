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
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseleftRearMotor;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaseleftFrontMotor;
std::shared_ptr<frc::SpeedControllerGroup> RobotMap::driveBaseLeftMotors;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaserightFrontMotor;
std::shared_ptr<frc::SpeedController> RobotMap::driveBaserightRearMotor;
std::shared_ptr<frc::SpeedControllerGroup> RobotMap::driveBaseRightMotors;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION


// These are "helper functions", intended to work around some problems in the code
// that Robot Builder currently generates.  (It's still generating code like it
// did for 2017 and earlier, but things are different in 2018.)
inline Jaguar* createMotor(int port, const char* const subsystem, const char* const name) {
	Jaguar* const motor = new Jaguar(port);
	motor->SetSubsystem(subsystem);
	motor->SetName(name);
	return motor;
}
inline SpeedControllerGroup* createSpeedControllerGroup(SpeedController& motor1, SpeedController& motor2, const char* const subsystem, const char* const name) {
	SpeedControllerGroup* const speedControllerGroup = new SpeedControllerGroup(motor1, motor2);
	speedControllerGroup->SetSubsystem(subsystem);
	speedControllerGroup->SetName(name);
	return speedControllerGroup;
}

// #define USE_ROBOT_BUILDER_INIT

void RobotMap::init() {
#if !defined(USE_ROBOT_BUILDER_INIT)
	// NOTE: This code needs to be kept in sync with the stuff in the "#else" clause, below.
	// This version works around some outdated code produced by Robot Builder 2018, but if
	// we ever make updates in RobotBuilder and re-export the C++ code, it will be out of
	// sync with those changes, and will have to be updated manually.
	//
	// (On the other hand, this version of the code won't produce warnings when compiled,
	// which means that it's easier to see what's *really* wrong when the compiler finds
	// something funky.)
	driveBaserightFrontMotor.reset(createMotor(0, "Drive Base", "rightFrontMotor"));
	driveBaserightRearMotor.reset(createMotor(3, "Drive Base", "rightRearMotor"));
    driveBaseleftFrontMotor.reset(createMotor(1, "Drive Base", "leftFrontMotor"));
    driveBaseleftRearMotor.reset(createMotor(2, "Drive Base", "leftRearMotor"));

    driveBaseLeftMotors.reset(createSpeedControllerGroup(*driveBaseleftFrontMotor, *driveBaseleftRearMotor, "DriveBase", "LeftDriveMotors"));
    driveBaseRightMotors.reset(createSpeedControllerGroup(*driveBaserightFrontMotor, *driveBaserightRearMotor, "DriveBase", "RightDriveMotors"));
#else
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    frc::LiveWindow *lw = frc::LiveWindow::GetInstance();

    driveBaseleftRearMotor.reset(new frc::PWMVictorSPX(2));
    lw->AddActuator("DriveBase", "leftRearMotor", std::static_pointer_cast<frc::PWMVictorSPX>(driveBaseleftRearMotor));
    
    driveBaseleftFrontMotor.reset(new frc::PWMVictorSPX(1));
    lw->AddActuator("DriveBase", "leftFrontMotor", std::static_pointer_cast<frc::PWMVictorSPX>(driveBaseleftFrontMotor));
    
    driveBaseLeftMotors = std::make_shared<frc::SpeedControllerGroup>(*driveBaseleftFrontMotor, *driveBaseleftRearMotor  );
    lw->AddActuator("DriveBase", "LeftMotors", driveBaseLeftMotors);
    
    driveBaserightFrontMotor.reset(new frc::PWMVictorSPX(0));
    lw->AddActuator("DriveBase", "rightFrontMotor", std::static_pointer_cast<frc::PWMVictorSPX>(driveBaserightFrontMotor));
    
    driveBaserightRearMotor.reset(new frc::PWMVictorSPX(3));
    lw->AddActuator("DriveBase", "rightRearMotor", std::static_pointer_cast<frc::PWMVictorSPX>(driveBaserightRearMotor));
    
    driveBaseRightMotors = std::make_shared<frc::SpeedControllerGroup>(*driveBaserightFrontMotor, *driveBaserightRearMotor  );
    lw->AddActuator("DriveBase", "RightMotors", driveBaseRightMotors);
    

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
#endif
}
