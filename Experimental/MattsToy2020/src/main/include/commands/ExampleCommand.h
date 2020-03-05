/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * A trivial sample command (which does nothing except log a fixed message).
 */
class ExampleCommand
    : public frc2::CommandHelper<frc2::CommandBase, ExampleCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param message the message to be logged when the command is triggered
   */
  explicit ExampleCommand(std::string message) : m_message(message) {
  }

  /// Logs the message associated with this command.
  void Initialize() override;

  /// Always returns true.
  bool IsFinished() override {
    return true;
  }

 private:
  std::string m_message;  ///< Message associated with this command.
};

/**
 * A trivial command that explicitly does nothing (other than print a message to
 * that effect when it's started).
 */
class DoNothingCommand : public ExampleCommand {
 public:
  DoNothingCommand() : ExampleCommand("Do nothing") {
  }
};
