// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootWithLimitSwitch.h"
#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

using std::cout; using std::endl;

ShootWithLimitSwitch::ShootWithLimitSwitch(Shooter* shooter, Intake* intake)
    : shooter(shooter), intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooter, intake});
}

// Called when the command is initially scheduled.
void ShootWithLimitSwitch::Initialize() {
  std::cout << "Initializing ShootWithLimitSwitch" << std::endl;
  shooter->setShootingMotor(.75);
  bool initialLimitSwitchState = intake->IsBallInChamber();
}

// Called repeatedly when this Command is scheduled to run
void ShootWithLimitSwitch::Execute() {
  std::cout << "Executing ShootWithLimitSwitch" << std::endl;
  struct timeval time_now{};
    gettimeofday(&time_now, nullptr);
    time_t msecs_time = (time_now.tv_sec * 1000) + (time_now.tv_usec / 1000);
    cout << "seconds since epoch: " << time_now.tv_sec << endl;
    cout << "milliseconds since epoch: "  << msecs_time << endl << endl;

  // Stage 1
  // Reference time = get current time
  // if ball not in chamber, advance conveyor
  Stage = 1;
  ReferenceTime = msecs_time;
  if (!(intake->IsBallInChamber())) {
    intake->ConveyBallOn();
  } else {
    intake->ConveyBallOff();
  }

  // Stage 2
  // wait until current time = reference time + X
  Stage = 2;
  if(msecs_time <= ReferenceTime + WaitTime) {
      std::cout << "It's not ready yet!" << std::endl;
  } else {
    std::cout << "Moving to Stage 3" << std::endl;
    // Stage 3
    Stage = 3;
    // feed ball to shooter
    // wait until limit switch is off
    if(intake->IsBallInChamber()) {
      intake->ConveyBallOn();
    }
  }

}

// Called once the command ends or is interrupted.
void ShootWithLimitSwitch::End(bool interrupted) {
  shooter->stopShootingMotor();
  intake->ConveyBallOff();
}
