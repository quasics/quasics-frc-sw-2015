// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

Drivebase::Drivebase() {
    SetName("Drivebase");

  m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
  m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

  m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));


}




// This method will be called once per scheduler run
void Drivebase::Periodic(){

}

void Drivebase::TankDrive(double leftPower, double rightPower) {
    m_drive->TankDrive(leftPower, rightPower);
}