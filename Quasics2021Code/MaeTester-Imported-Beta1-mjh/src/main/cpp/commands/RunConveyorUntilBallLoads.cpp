// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunConveyorUntilBallLoads.h"

#include <iostream>

RunConveyorUntilBallLoads::RunConveyorUntilBallLoads(Intake *intake,
                                                     double conveyorPower,
                                                     double pickupPower,
                                                     units::second_t timeout)
    : m_intake(intake),
      m_conveyorPower(conveyorPower),
      m_pickupPower(pickupPower),
      m_timeout(timeout)
{
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void RunConveyorUntilBallLoads::Initialize()
{
  std::cout << "Timeout on ball movement = " << m_timeout.value() << std::endl;
  if (m_intake->IsBallInChamber())
  {
    m_state = eStartingBallSensed;
    std::cout << "Setting state to 'starting ball sensed'" << std::endl;
  }
  else
  {
    m_state = eNoBallSensed;
    std::cout << "Setting state to 'no ball sensed'" << std::endl;
  }
  m_intake->SetConveyorSpeed(Intake::MOTOR_SLOW_POWER);

  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void RunConveyorUntilBallLoads::Execute()
{
  if (m_state == eStartingBallSensed)
  {
    // OK, have we kicked the current ball up towards the shooter?
    if (!m_intake->IsBallInChamber())
    {
      m_state = eNoBallSensed;
      std::cout << "Setting state to 'no ball sensed'" << std::endl;
    }
  }
  else if (m_state == eNoBallSensed)
  {
    // OK, has a ball been loaded into the chamber?
    if (m_intake->IsBallInChamber())
    {
      m_state = eNewBallSensed;
      std::cout << "Setting state to 'new ball sensed'" << std::endl;
    }
    // Note that we're assuming that the sensor is mounted in a position where
    // it won't miss the transitions if the balls are loading immediately after
    // each other (e.g., along the center line of the ball path, where two balls
    // more-or-less butting up against each other won't provide a gap to be
    // sensed).
    //
    // But there's another tricky bit below, too....
  }
  else if (m_state == eNewBallSensed)
  {
    // Here's the tricky bit.  Depending on how long it's been since the first
    // ball "left" the sensor, it might not be up at the shooter yet.
    //
    // One option would be to tune things, so that we wait some given amount of
    // time after we sense the new ball before deciding, "We're done!", without
    // waiting *so* long that the new ball leaves the sensor area.
    //
    // Another (maybe better) option would be to wait until the *new* ball left
    // the sensor, and then stop immediately.
    //
    // For the  moment (as a initial/trivial approach), we'll assume that 1
    // extra cycle (i.e., 0.02 sec) is enough to give us the cover we need, but
    // it seems doubtful that this will be the case.
    m_state = eFinished;
    std::cout << "Setting state to 'finished'" << std::endl;
  }
}

// Called once the command ends or is interrupted.
void RunConveyorUntilBallLoads::End(bool interrupted)
{
  m_intake->SetBallPickupSpeed(Intake::MOTOR_OFF_POWER);
  m_intake->SetConveyorSpeed(Intake::MOTOR_OFF_POWER);
}

// Returns true when the command should end.
bool RunConveyorUntilBallLoads::IsFinished()
{
  return (m_state == eFinished) || m_timer.HasElapsed(m_timeout);
}
