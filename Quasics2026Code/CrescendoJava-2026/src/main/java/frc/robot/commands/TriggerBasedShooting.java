// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class TriggerBasedShooting extends Command {
  private final Shooter m_shooter;
  private final XboxController m_XboxController;
  /** Creates a new TriggerBasedShooting. */
  public TriggerBasedShooting(Shooter shooter, XboxController xboxController) {
    m_shooter = shooter;
    m_XboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_XboxController.getLeftTriggerAxis() > 0.5) {
      m_shooter.setFlywheelSpeed(.25);
    } else if (m_XboxController.getRightTriggerAxis() > 0.5) {
      m_shooter.setFlywheelSpeed(.75);
    } else {
      m_shooter.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }
}
