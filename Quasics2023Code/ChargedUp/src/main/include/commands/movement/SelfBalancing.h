// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/Drivebase.h"

#define INITIAL_POWER_BOOST_FOR_BALANCE

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 *
 */

// Uses PID controll to sense the angle of the robot and adjust the movement so
// the robot becomes level
class SelfBalancing
    : public frc2::CommandHelper<frc2::CommandBase, SelfBalancing> {
 public:
  SelfBalancing(Drivebase* drivebase);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

 private:
  // Coefficient to change direction of power given to motors based of
  // m_pastAngle m_pastAngle > 0 positive m_pastAngle < 0 negative
  int GetPowerSignFromAngle(double angle) {
    if (angle > 0) {
      return 1;
    } else {
      return -1;
    }
  }

 private:
  Drivebase* m_drivebase;
  // Used to calculate the power to give to the wheels
  frc2::PIDController m_pid{SelfBalancingConstants::PID::kP,
                            SelfBalancingConstants::PID::kI,
                            SelfBalancingConstants::PID::kD};
  // Saves the angle that the robot was on for subsequent slope
  // identification(see slopeOfRamp)
  double m_pastAngle;
  // flag that turns off the FeedForward command(a.k.a balancing before PID
  // Controller)
  bool m_noFeedFowardPower = false;
  // flag that tells the robot when to begin finetuning the balancing with PID
  // controller
  bool m_activatePID = false;

  const double ANGLE_FOR_PID_ACTIVATION = 2.0;

#ifdef INITIAL_POWER_BOOST_FOR_BALANCE
  const double ANGLE_FOR_PHASE_CHANGE = 20.0;
#endif
};
