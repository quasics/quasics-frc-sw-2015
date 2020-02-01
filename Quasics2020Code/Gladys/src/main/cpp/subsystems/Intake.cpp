/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Intake.h"

constexpr int BallIntakeMotor = 5;
constexpr int IntakeArmMotor = 6;

Intake::Intake()


: BallIntake(BallIntakeMotor, rev::CANSparkMax::MotorType::kBrushless),
  IntakeArm(IntakeArmMotor, rev::CANSparkMax::MotorType::kBrushless){}

// This method will be called once per scheduler run
void Intake::Periodic() {}
    void Intake::TurnSuctionOn () {
        BallIntake.Set(0.5);
    }
    void Intake::TurnSuctionOff () {
        BallIntake.Set(0);
    }
    void Intake::RotateShoulderUp() {
        IntakeArm.Set(0.5);
    }
    void Intake::RotateShoulderDown() {
        IntakeArm.Set(-0.5);
    }
    void Intake::TurnShoulderOff() {
        IntakeArm.Set(0);
    }