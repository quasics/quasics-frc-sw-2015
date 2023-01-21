// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/SelfBalancing.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
  // SelfBalancing SelfBalancing(&m_driveTrain);
  //.SetDefaultCommand(SelfBalancing);
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  // SelfBalancing selfBalancer(&m_driveTrain);
  // m_driveTrain.SetDefaultCommand(selfBalancer);

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this]
                { return m_subsystem.ExampleCondition(); })
      .OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
  // std::unique_ptr<frc2::CommandBase> ptr(new SelfBalancing(&m_driveTrain));
  return frc2::CommandPtr(                // Build a CommandPtr object from....
    std::unique_ptr<frc2::CommandBase>(   // a std::unique_ptr to CommandBase, which wraps....
      new SelfBalancing(&m_driveTrain)    // the command we want to have executed.
    )
  );
}
