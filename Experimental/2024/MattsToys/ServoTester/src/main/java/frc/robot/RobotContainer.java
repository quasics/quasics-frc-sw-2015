// Copyright (c) Matt Healy, Quasics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ServoPositionerCmd;
import frc.robot.subsystems.ServoHost;

public class RobotContainer {
  final ServoHost m_servoHost;
  XboxController m_controller = new XboxController(1);

  public RobotContainer() {
    m_servoHost = new ServoHost();
    configureBindings();
  }

  private void configureBindings() {
    m_servoHost.setDefaultCommand(new ServoPositionerCmd(m_servoHost, () -> m_controller.getLeftY()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

/*
 In C++, this might look roughly like:

 // RobotContainer.h
 #pragma once

 // Some #includes go here.....

 class RobotContainer {
 private:
  ServoHost m_servoHost;
  XboxController m_controller;
 public:
  RobotContainer();

  Command getAutonomousCommand();

 private:
  void configureBindings();
};

 // In RobotContainer.cpp
 #include "RobotContainer.h"
 // Other #includes.....

 RobotContainer::RobotContainer() : m_controller(1) {
  configureBindings();
 }
 .....



 */