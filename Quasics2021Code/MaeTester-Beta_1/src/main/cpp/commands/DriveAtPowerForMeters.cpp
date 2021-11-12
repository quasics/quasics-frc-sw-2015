// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveAtPowerForMeters.h"

#include <cmath>

DriveAtPowerForMeters::DriveAtPowerForMeters(
    Drivebase* drivebase, double power, units::length::meter_t desiredMeters)
    : drivebase(drivebase),
      power(
          (desiredMeters.to<double>() / std::abs(desiredMeters.to<double>())) *
          std::abs(power)),
      desiredMeters(desiredMeters) {
  AddRequirements({drivebase});
}

// Called when the command is initially scheduled.
void DriveAtPowerForMeters::Initialize() {
  startingMeters = (drivebase->GetLeftEncoderDistance() +
                    drivebase->GetRightEncoderDistance()) /
                   2;
  destination = startingMeters + desiredMeters;
  drivebase->SetMotorSpeed(power, power);
  std::cout << "desiredMeters = " << desiredMeters.value()
            << ", startingMeters = " << startingMeters.value()
            << ", destination = " << destination.value() << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void DriveAtPowerForMeters::Execute() {
  drivebase->SetMotorSpeed(
      power, power);  // Note for later: figure out why we need this
}

// Called once the command ends or is interrupted.
void DriveAtPowerForMeters::End(bool interrupted) {
  drivebase->Stop();
}

// Returns true when the command should end.
bool DriveAtPowerForMeters::IsFinished() {
  auto currentReading = (drivebase->GetLeftEncoderDistance() +
                         drivebase->GetRightEncoderDistance()) /
                        2;
  std::cout << "desiredMeters = " << desiredMeters.value()
            << ", startingMeters = " << startingMeters.value()
            << ", destination = " << destination.value()
            << ", currentReading = " << currentReading.value() << std::endl;
  // if desires < 0
  if (desiredMeters.to<double>() < 0) {
    if (currentReading.to<double>() <= destination.to<double>()) {
      return true;
    }
    return false;
  } else {
    if (currentReading.to<double>() >= destination.to<double>()) {
      return true;
    }
    return false;
  }
}