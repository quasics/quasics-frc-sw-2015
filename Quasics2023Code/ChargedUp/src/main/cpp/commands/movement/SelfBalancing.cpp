// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/SelfBalancing.h"

#include <iostream>

#undef Feed_Forward_Scaling
SelfBalancing::SelfBalancing(Drivebase* drivebase) : m_drivebase(drivebase) {
  AddRequirements(drivebase);
  SetName("SelfBalancing");
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  // std::cout << "Begginnning to Self Balance" << std::endl;
  m_noFeedFowardPower = false;
  m_activatePID = false;
  m_pid.Reset();
  m_pid.SetTolerance(2.0, 0);

  const double pitchUpDegrees =
      m_drivebase->GetPitch() * -1;  // Pitch up is neg pitch down is
                                     // pos

  m_pastAngle = pitchUpDegrees;
  const int slopeOfRamp = GetPowerSignFromAngle(pitchUpDegrees);

  m_drivebase->TankDrive(slopeOfRamp * 0.4, slopeOfRamp * 0.4);
}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  // std::cout << "Self Balancing" << std::endl;
  const double currentAngle =
      m_drivebase->GetPitch() * -1;  // GYRO ON GLADYS IS GIVING ROLL AS UP AND
                                     // DOWN ANGLE NEGATED BECAUSE OF VALUES
                                     // Negative up positive down

  double power = 0.0;
  if (m_noFeedFowardPower == false) {
#ifdef Feed_Forward_Scaling
    if (std::abs(currentAngle) > ANGLE_FOR_PHASE_CHANGE) {
      power = 0.6;
    } else {
      power = 0.4;
    }
#else
    power = 0.4;
#endif
    if (std::abs(currentAngle) <= ANGLE_FOR_PID_ACTIVATION) {
      m_noFeedFowardPower = true;
      m_activatePID = true;
    }
  }

  if (m_activatePID) {
    power = m_pid.Calculate(currentAngle, 0.0) * -1;
  } else {
    const int slopeOfRamp = GetPowerSignFromAngle(currentAngle);
    power *= slopeOfRamp;
  }

  m_drivebase->TankDrive(power, power);
  // m_pastAngle = currentAngle;
}

// Called once the command ends or is interrupted.
void SelfBalancing::End(bool interrupted) {
  // std::cout << "Ending Balancing" << std::endl;
  m_drivebase->Stop();
}

// FOR TESTING PURPOSES HAVE IT REPORTING THE GYRO ANGLE TO THE SMART
// DASHBOARD