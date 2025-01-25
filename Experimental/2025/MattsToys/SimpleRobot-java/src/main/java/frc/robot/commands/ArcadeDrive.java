// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ArcadeDrive extends Command {
  final private IDrivebase m_drivebase;
  final private Supplier<Double> m_speed;
  final private Supplier<Double> m_rotation;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(IDrivebase drivebase, Supplier<Double> speed, Supplier<Double> rotation) {
    this.m_drivebase = drivebase;
    this.m_speed = speed;
    this.m_rotation = rotation;

    if (drivebase != null) {
      addRequirements(drivebase.asSubsystem());
    }
  }

  private void updateSpeeds() {
    m_drivebase.arcadeDrive(
        IDrivebase.MAX_SPEED.times(m_speed.get()), IDrivebase.MAX_ROTATION.times(m_rotation.get()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateSpeeds();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
