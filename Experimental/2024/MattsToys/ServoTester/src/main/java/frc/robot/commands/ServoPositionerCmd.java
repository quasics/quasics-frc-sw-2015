// Copyright (c) Matt Healy, Quasics, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ServoHost;

public class ServoPositionerCmd extends Command {
  final ServoHost m_host;
  Supplier<Double> m_positionSupplier;

  /** Creates a new ServoPositionerCmd. */
  public ServoPositionerCmd(ServoHost host, Supplier<Double> positionSupplier) {
    m_host = host;
    m_positionSupplier = positionSupplier;
    addRequirements(m_host);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    update();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_host.setServoPosition(0);
  }

  private void update() {
    m_host.setServoPosition(m_positionSupplier.get());
  }
}
