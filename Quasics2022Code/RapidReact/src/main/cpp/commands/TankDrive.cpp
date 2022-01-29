// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"
#include "Constants.h"

TankDrive::TankDrive(Drivebase* drivebase, frc::Joystick* driverStick) : m_drivebase(drivebase), m_driverStick(driverStick){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  UpdateSpeeds();
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  UpdateSpeeds();
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {
  m_drivebase->Stop();
}

void TankDrive::UpdateSpeeds() {
  double leftPower = m_driverStick->GetRawAxis(OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
  double rightPower = m_driverStick->GetRawAxis(OperatorInterface::LogitechGamePad::RIGHT_Y_AXIS);
  m_drivebase->SetMotorPower(leftPower, rightPower);
}