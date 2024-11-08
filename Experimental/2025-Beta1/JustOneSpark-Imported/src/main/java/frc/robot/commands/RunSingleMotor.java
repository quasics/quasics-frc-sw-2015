// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SingleMotor;

public class RunSingleMotor extends Command {
  /** Creates a new RunSingleMotor. */
  SingleMotor m_singleMotor;
  public RunSingleMotor(SingleMotor singleMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_singleMotor = singleMotor;
    addRequirements(singleMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_singleMotor.setPower(0.4);
    System.out.println("Running single motor");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running single motor");
    m_singleMotor.setPower(0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending single motor");
    m_singleMotor.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
