// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/SelfBalancing.h"

#undef ENABLE_FEED_FORWARD_SCALING

SelfBalancing::SelfBalancing(Drivebase* drivebase) : m_drivebase(drivebase) {
  AddRequirements(drivebase);
  SetName("SelfBalancing");
}

// Called when the command is initially scheduled.
void SelfBalancing::Initialize() {
  m_noFeedFowardPower = false;
  m_activatePID = false;
  m_pid.Reset();
  // CODE_REVIEW(matthew): Avoid magic numbers.  What do these values mean, and
  // how were they established?
  m_pid.SetTolerance(2.0, 0);

  const double pitchUpDegrees =
      m_drivebase->GetPitch() * -1;  // Pitch up is neg pitch down is
                                     // pos

  m_pastAngle = pitchUpDegrees;
  const int slopeOfRamp = GetPowerSignFromAngle(pitchUpDegrees);

  // CODE_REVIEW(matthew): Avoid magic numbers.  What do these values mean, and
  // how were they established?
  m_drivebase->TankDrive(slopeOfRamp * 0.4, slopeOfRamp * 0.4);
}

// Called repeatedly when this Command is scheduled to run
void SelfBalancing::Execute() {
  const double currentAngle =
      m_drivebase->GetPitch() * -1;  // GYRO ON GLADYS IS GIVING ROLL AS UP AND
                                     // DOWN ANGLE NEGATED BECAUSE OF VALUES
                                     // Negative up positive down

  // CODE_REVIEW(matthew): Get rid of "magic numbers" in here, like 0.4, 0.6,
  // etc.
  double power = 0.0;
  if (!m_noFeedFowardPower) {
#ifdef ENABLE_FEED_FORWARD_SCALING
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

  // Figure out the power to be applied to the motors.
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
  m_drivebase->Stop();
}

// FOR TESTING PURPOSES HAVE IT REPORTING THE GYRO ANGLE TO THE SMART
// DASHBOARD