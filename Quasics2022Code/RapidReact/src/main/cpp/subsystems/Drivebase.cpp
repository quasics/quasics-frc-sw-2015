// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"
#include <rev/CANSparkMax.h>
#include <iostream>
#include <wpi/numbers>

Drivebase::Drivebase(){
 m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
 m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

 m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));
}

// This method will be called once per scheduler run
void Drivebase::Periodic() {

}
//Todo - Write code for the following 4 commands

void Drivebase::SetMotorPower(double leftPower, double rightPower){
    m_drive->TankDrive(leftPower, rightPower);
}

units::meter_t  Drivebase::GetLeftEncoders(){

    return units::meter_t (m_leftFrontEncoder.GetPosition());
}

units::meter_t  Drivebase::GetRightEncoders(){

    return units::meter_t (m_rightFrontEncoder.GetPosition());
}

void Drivebase::ResetEncoders(){
    m_leftFrontEncoder.SetPosition(0);
    m_rightFrontEncoder.SetPosition(0);
    m_leftBackEncoder.SetPosition(0);
    m_rightBackEncoder.SetPosition(0);
}