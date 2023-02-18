// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SelfBalancing.h"

#include <iostream>

SelfBalancing::SelfBalancing(Drivebase* drivebase) : m_drivebase(drivebase) {
  AddRequirements(drivebase);
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  noFeedFowardPower = false;
  activatePID = false;
  pid.Reset();
  pid.SetTolerance(2.0, 0);
  pastAngle = m_drivebase->GetPitch();  // ADJUST FOR NOT FLAT GYRO
  if ((pastAngle) > 0) {
    slopeOfRamp = 1;
  }
  if (pastAngle < 0) {
    slopeOfRamp = -1;
  }

  m_drivebase->TankDrive(slopeOfRamp * 0.4, slopeOfRamp * 0.4);
}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  double currentAngle = m_drivebase->GetPitch();  // ADJUST FOR NOT FLAT GYRO
  double power = 0.0;
  if (noFeedFowardPower == false) {
    power = 0.4;
    if (currentAngle > -2.0 and currentAngle < 2.0) {
      noFeedFowardPower = true;
      activatePID = true;
    }
  }
  if (activatePID) {
    power = pid.Calculate(currentAngle, 0.0);
  }

  if ((pastAngle) > 0) {
    slopeOfRamp = 1;
  }
  if (pastAngle < 0) {
    slopeOfRamp = -1;
  }

  if (!activatePID) {
    m_drivebase->TankDrive(slopeOfRamp * power, slopeOfRamp * power);
  }
  if (activatePID) {
    m_drivebase->TankDrive(power * -1, power * -1);
  }
  pastAngle = currentAngle;
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) { m_drivebase->Stop(); }

// FOR TESTING PURPOSES HAVE IT REPORTING THE GYRO ANGLE TO THE SMART DASHBOARD