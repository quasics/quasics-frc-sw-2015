// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StraightLineDriving.h"

// When button pressed, read current angle, in execute
// pid.calculate(currentAngle, to original reading) Use arcade drive give use
// forward power as what is given by joystick
// GetRawAxis(OperatorInterface::LogitechGamePad::LEFT_Y_AXIS)

StraightLineDriving::StraightLineDriving(Drivebase* drivebase,
                                         frc::Joystick* driverStick)
    : m_drivebase(drivebase), m_driverStick(driverStick) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void StraightLineDriving::Initialize() {
  originalAngle = m_drivebase->GetAngle();
  robotSpeed = m_driverStick->GetRawAxis(
      OperatorInterface::LogitechGamePad::LEFT_Y_AXIS);
}

// Called repeatedly when this Command is scheduled to run
void StraightLineDriving::Execute() {
  currentAngle = m_drivebase->GetAngle();
  double rotationCorrection =
      pid.Calculate(currentAngle.value(), originalAngle.value());
  m_drivebase->ArcadeDrive(robotSpeed, rotationCorrection);
}

// Called once the command ends or is interrupted.
void StraightLineDriving::End(bool interrupted) {
  m_drivebase->ArcadeDrive(robotSpeed, 0);
}

// Returns true when the command should end.
bool StraightLineDriving::IsFinished() {
  return false;
}
