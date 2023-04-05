// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/SelfBalancing.h"

#include <iostream>

SelfBalancing::SelfBalancing(Drivebase* drivebase) : m_drivebase(drivebase) {
  AddRequirements(drivebase);
  SetName("SelfBalancing");
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  // std::cout << "Begginnning to Self Balance" << std::endl;
  noFeedFowardPower = false;
  activatePID = false;
  pid.Reset();
  pid.SetTolerance(2.0, 0);
  pastAngle = m_drivebase->GetPitch() * -1;  // Pitch up is neg pitch down is
                                             // pos
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
  // std::cout << "Self Balancing" << std::endl;
  double currentAngle =
      m_drivebase->GetPitch() * -1;  // GYRO ON GLADYS IS GIVING ROLL AS UP AND
                                     // DOWN ANGLE NEGATED BECAUSE OF VALUES
                                     // Negative up positive down
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
void SelfBalancing::End(bool interrupted) {
  // std::cout << "Ending Balancing" << std::endl;
  m_drivebase->Stop();
}

// FOR TESTING PURPOSES HAVE IT REPORTING THE GYRO ANGLE TO THE SMART DASHBOARD