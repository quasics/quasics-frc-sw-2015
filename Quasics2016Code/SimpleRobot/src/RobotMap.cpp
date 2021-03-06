// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "RobotMap.h"
#include "LiveWindow/LiveWindow.h"


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
std::shared_ptr<SpeedController> RobotMap::driveSystemLeftFront;
std::shared_ptr<SpeedController> RobotMap::driveSystemLeftRear;
std::shared_ptr<SpeedController> RobotMap::driveSystemRightFront;
std::shared_ptr<SpeedController> RobotMap::driveSystemRightRear;
std::shared_ptr<Encoder> RobotMap::driveSystemLeftEncoder;
std::shared_ptr<Encoder> RobotMap::driveSystemRightEncoder;
std::shared_ptr<SpeedController> RobotMap::intakeLeftIntakeWheel;
std::shared_ptr<SpeedController> RobotMap::intakeRightIntakeWheel;
std::shared_ptr<DoubleSolenoid> RobotMap::intakePusher;
std::shared_ptr<SpeedController> RobotMap::intakeArmLeftArm;
std::shared_ptr<SpeedController> RobotMap::intakeArmRightArm;
std::shared_ptr<Encoder> RobotMap::intakeArmLeftArmEncoder;
std::shared_ptr<Encoder> RobotMap::intakeArmRightArmEncoder;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION

void RobotMap::init() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    LiveWindow *lw = LiveWindow::GetInstance();

    driveSystemLeftFront.reset(new Talon(0));
    lw->AddActuator("DriveSystem", "LeftFront", (Talon&) driveSystemLeftFront);

    driveSystemLeftRear.reset(new Talon(1));
    lw->AddActuator("DriveSystem", "LeftRear", (Talon&) driveSystemLeftRear);

    driveSystemRightFront.reset(new Talon(2));
    lw->AddActuator("DriveSystem", "RightFront", (Talon&) driveSystemRightFront);

    driveSystemRightRear.reset(new Talon(3));
    lw->AddActuator("DriveSystem", "RightRear", (Talon&) driveSystemRightRear);

    driveSystemLeftEncoder.reset(new Encoder(0, 1, true, Encoder::k4X));
    lw->AddSensor("DriveSystem", "Left Encoder", driveSystemLeftEncoder);
    driveSystemLeftEncoder->SetDistancePerPulse(4.318E-4);
    driveSystemLeftEncoder->SetPIDSourceType(PIDSourceType::kRate);
    driveSystemRightEncoder.reset(new Encoder(2, 3, false, Encoder::k4X));
    lw->AddSensor("DriveSystem", "RightEncoder", driveSystemRightEncoder);
    driveSystemRightEncoder->SetDistancePerPulse(4.318E-4);
    driveSystemRightEncoder->SetPIDSourceType(PIDSourceType::kRate);
    intakeLeftIntakeWheel.reset(new Victor(4));
    lw->AddActuator("Intake", "LeftIntakeWheel", (Victor&) intakeLeftIntakeWheel);

    intakeRightIntakeWheel.reset(new Victor(5));
    lw->AddActuator("Intake", "RightIntakeWheel", (Victor&) intakeRightIntakeWheel);

    intakePusher.reset(new DoubleSolenoid(1, 0, 1));

    intakeArmLeftArm.reset(new Spark(6));
    lw->AddActuator("IntakeArm", "LeftArm", (Spark&) intakeArmLeftArm);

    intakeArmRightArm.reset(new Spark(7));
    lw->AddActuator("IntakeArm", "RightArm", (Spark&) intakeArmRightArm);

    intakeArmLeftArmEncoder.reset(new Encoder(4, 5, false, Encoder::k4X));
    lw->AddSensor("IntakeArm", "LeftArmEncoder", intakeArmLeftArmEncoder);
    intakeArmLeftArmEncoder->SetDistancePerPulse(1.0);
    intakeArmLeftArmEncoder->SetPIDSourceType(PIDSourceType::kRate);
    intakeArmRightArmEncoder.reset(new Encoder(6, 7, true, Encoder::k4X));
    lw->AddSensor("IntakeArm", "RightArmEncoder", intakeArmRightArmEncoder);
    intakeArmRightArmEncoder->SetDistancePerPulse(1.0);
    intakeArmRightArmEncoder->SetPIDSourceType(PIDSourceType::kRate);


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}
