/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Commands/AutoLighting.h"
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <iostream>
#include "Robot.h"

AutoLighting::AutoLighting() {
  // Use Requires() here to declare subsystem dependencies
  Requires(Robot::lighting.get());
}


// Called repeatedly when this Command is scheduled to run
void AutoLighting::Execute() {
  auto& driverStation = frc::DriverStation::GetInstance();

  if (!driverStation.IsDSAttached()) {
    // Send command to signal that the Driver Station isn't connected (yet).
    transmitMode("no-ds");
    return;
  }

  const auto ds_alliance = driverStation.GetAlliance();
  const auto elevatorMode = Robot::GetLifterMode();
  
  std::string color = "default";
  switch (ds_alliance) {
    case frc::DriverStation::kRed:
      color = "red";
      break;
    case frc::DriverStation::kBlue:
      color = "blue";
      break;
    case frc::DriverStation::kInvalid:
      color = "demo";
      break;
  }

  std::string mode = "default";
  if(elevatorMode == Robot::eBoth){
    mode = "Both";
  }
  else if(elevatorMode == Robot::eElevator){
    mode = "Elevator";
  }
  else if(elevatorMode == Robot::eLifter){
    mode = "Lifter";
  }

  transmitMode(mode);
  transmitColor(color);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoLighting::IsFinished() { return false; }

void AutoLighting::Initialize() {
  lastColorCommand = "";
  lastModeCommand = "";
}

void AutoLighting::transmitMode(std::string modeCommand, bool alwaysSend) {
  if (alwaysSend || modeCommand != lastModeCommand) {
    std::cout << "Sending command: '" << modeCommand << "'" << std::endl;
    // If the command (state) has changed, or if we're forcing a re-send, pass
    // the current status on to the Arduino.
    if (!Robot::lighting->sendCommandToArduino(modeCommand)) {
      std::cerr << "--- Error sending command!\n";
    }

    lastModeCommand = modeCommand;
  }
}

void AutoLighting::transmitColor(std::string colorCommand, bool alwaysSend) {
  if (alwaysSend || colorCommand != lastColorCommand) {
    std::cout << "Sending command: '" << colorCommand << "'" << std::endl;
    // If the command (state) has changed, or if we're forcing a re-send, pass
    // the current status on to the Arduino.
    if (!Robot::lighting->sendCommandToArduino(colorCommand)) {
      std::cerr << "--- Error sending command!\n";
    }
    lastColorCommand = colorCommand;
  }
}
