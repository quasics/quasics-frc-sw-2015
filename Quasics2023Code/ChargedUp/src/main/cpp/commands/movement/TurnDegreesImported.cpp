// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/movement/TurnDegreesImported.h"

#include <iostream>

#define CURRENT_TURN_VERSION
#undef FASTER_SLOW_DOWN
#undef NO_SLOW_DOWN

TurnDegreesImported::TurnDegreesImported(Drivebase* drivebase, double speed,
                                         units::degree_t angle)
    : m_drivebase(drivebase),
      m_speed(angle > 0_deg ? std::abs(speed) : -std::abs(speed)),
      m_angle(angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drivebase);
  SetName("TurnDegreesImported");
}

// Called when the command is initially scheduled.
void TurnDegreesImported::Initialize() {
  /*#ifdef CURRENT_TURN_VERSION
    angleTest = m_angle * DecreaseForCompensation;
    std::cout << "AngleTestValue" << angleTest.value() << std::endl;
    std::cout << "Wanted Turn" << m_angle.value() << std::endl;
    multiplier = 1.0;
    turningleft = angleTest > 0_deg;  // substitution


    multiplier = 1.00;
    startingposition = m_drivebase->GetYaw();
    m_drivebase->SetBrakingMode(true);
    m_drivebase->TankDrive(-1 * m_speed, m_speed);
  #endif
  #ifdef NO_SLOW_DOWN
    startingposition = m_drivebase->GetYaw();
    turningleft = m_angle > 0_deg;
    m_drivebase->SetBrakingMode(true);
    m_drivebase->TankDrive(-1 * m_speed, m_speed);
  #endif
  */

  // if angle is greater than o and turning left no problem, if angle less than
  // 0 and turning right no problem, if angle is greater than 0 turning right
  // and less than 90 problem if angle is less than 0 and turning left and
  // greater than -90 problem
  subtraction = 0;
  startingposition = m_drivebase->GetYaw();
  m_drivebase->SetBrakingMode(true);
  m_drivebase->TankDrive(-1 * m_speed, m_speed);
}

// Called repeatedly when this Command is scheduled to run
void TurnDegreesImported::Execute() {
  units::degree_t currentPosition = m_drivebase->GetYaw();
  if (std::abs((startingposition + m_angle - currentPosition).value()) < 52.5 &&
      (std::abs(m_speed) > 0.3)) {
    subtraction = std::abs(m_speed) - 0.3;
    std::cout << "Invoking subtraction" << std::endl;
  }
  m_drivebase->SetBrakingMode(true);
  if (m_speed > 0) {
    m_drivebase->TankDrive(-1 * (m_speed - subtraction),
                           (m_speed - subtraction));
  } else {
    m_drivebase->TankDrive(-1 * (m_speed + subtraction),
                           (m_speed + subtraction));
  }
  /*
    #ifdef CURRENT_TURN_VERSION
      m_drivebase->SetBrakingMode(true);
      units::degree_t currentPosition = m_drivebase->GetYaw();
      if (turningleft) {
        if (currentPosition > ((startingposition + angleTest) * 0.5) &&
            (m_speed * multiplier > 0.5)) {
    #ifdef FASTER_SLOW_DOWN
          multiplier = multiplier * 0.90;
    #endif
          multiplier = multiplier * 0.95;
        }
        m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
      } else {
        if (currentPosition < ((startingposition + angleTest) * 0.5) &&
            (m_speed * multiplier > 0.5)) {
    #ifdef FASTER_SLOW_DOWN
          multiplier = multiplier * 0.90;
    #endif
          multiplier = multiplier * 0.95;
        }
        m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
      }


     if (m_angle > 100_deg) {
       m_drivebase->SetBrakingMode(true);
       units::degree_t currentPosition = m_drivebase->GetYaw();
       if (turningleft) {
         if (currentPosition > ((startingposition + angleTest) * 0.5) &&
             (m_speed * multiplier > 0.48)) {
           multiplier = multiplier * 0.95;
         }
         m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed *
  multiplier); } else { if (currentPosition < ((startingposition + angleTest) *
  0.5) && (m_speed * multiplier > 0.48)) { multiplier = multiplier * 0.95;
         }
         m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed *
  multiplier);
       }
     } else {
       m_drivebase->SetBrakingMode(true);
       units::degree_t currentPosition = m_drivebase->GetYaw();
       if (turningleft) {
         if (currentPosition > ((startingposition + angleTest) * 0.5) &&
             (m_speed * multiplier > 0.35)) {
           multiplier = multiplier * 0.95;
         }
         m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed *
  multiplier); } else { if (currentPosition < ((startingposition + angleTest) *
  0.5) && (m_speed * multiplier > 0.35)) { multiplier = multiplier * 0.95;
         }
         m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed *
  multiplier);
       }

    // chopping down to minimum speed 15 deg before destination

    // if turning left degrees are getting larger
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (turningleft) {
      if (currentPosition > (startingposition + m_angle - 15_deg)) {
        multiplier = 0.3;
      }
      m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
    } else {
      if (currentPosition < ((startingposition + angleTest + 15_deg))) {
        multiplier = 0.3;
      }
      m_drivebase->TankDrive(-1 * m_speed * multiplier, m_speed * multiplier);
    }

  // #endif
  #ifdef NO_SLOW_DOWN
    m_drivebase->SetBrakingMode(true);
    m_drivebase->TankDrive(-1 * m_speed, m_speed);
  #endif

  */
}

// Called once the command ends or is interrupted.
void TurnDegreesImported::End(bool interrupted) {
  m_drivebase->SetBrakingMode(true);
  m_drivebase->Stop();
}
// Returns true when the command should end.
bool TurnDegreesImported::IsFinished() {
  if (std::abs(m_angle.value()) == 90) {
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (m_angle > 0_deg) {
      if (currentPosition > startingposition + m_angle - 5_deg) {  // was 35
        return true;
      }
    } else {
      if (currentPosition < startingposition + m_angle + 5_deg) {
        return true;
      }
    }
    return false;
  } else {
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (m_angle > 0_deg) {
      if (currentPosition > startingposition + m_angle - 12.5_deg) {
        return true;
      }
    } else {
      if (currentPosition < startingposition + m_angle + 12.5_deg) {
        return true;
      }
    }
    return false;
  }
  return false;

  /*
  #ifdef CURRENT_TURN_VERSION
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (turningleft) {
      if (currentPosition >= (startingposition + m_angle - 10_deg)) {
        std::cout << "Difference Left "
                  << (currentPosition - (startingposition)).value() <<
  std::endl; return true; } else { return false;
      }
    } else {
      if (currentPosition <= (startingposition + m_angle - 10_deg)) {
        std::cout << "Difference Right "
                  << (currentPosition - (startingposition)).value() <<
  std::endl; return true; } else { return false;
      }
    }

    return false;
  #endif

  #ifdef NO_SLOW_DOWN
    units::degree_t currentPosition = m_drivebase->GetYaw();
    if (turningleft) {
      if (currentPosition >=
          (startingposition + m_angle) -
              15_deg) {  // arbitrary value to have time to stop turning
        return true;
      }
      return false;
    } else {
      if (currentPosition <=
          (startingposition + m_angle) +
              15_deg) {  // arbitrary value to have time to stop turning
        return true;
      }
      return false;
    }
  #endif*/
}
