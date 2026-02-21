// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

// () -> someButton.isPressed()
// () -> { code .... }

public class TriggerDrivenShootingCommand extends Command {
  private final Shooter m_shooter;
  private final double m_highSpeed;
  private final double m_lowSpeed;
  private final Supplier<Boolean> m_runHighSpeed;
  private final Supplier<Boolean> m_runLowSpeed;

  /** Creates a new TriggerDrivenShootingCommand. */
  public TriggerDrivenShootingCommand(Shooter shooter, double highSpeed,
      double lowSpeed, Supplier<Boolean> runHighSpeed,
      Supplier<Boolean> runLowSpeed) {
    m_shooter = shooter;
    m_highSpeed = highSpeed;
    m_lowSpeed = lowSpeed;
    m_runHighSpeed = runHighSpeed;
    m_runLowSpeed = runLowSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_runHighSpeed.get()) {
      m_shooter.SetSpeed(m_highSpeed);
    } else if (m_runLowSpeed.get()) {
      m_shooter.SetSpeed(m_lowSpeed);
    } else {
      m_shooter.Stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Stop();
  }
}