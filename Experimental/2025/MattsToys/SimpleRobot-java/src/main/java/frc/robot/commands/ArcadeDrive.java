// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;
import java.util.function.Supplier;

/**
 * Implments "arcade drive" handling, where one joystick controls
 * linear (forward/backward) velocity, and the other controls rotation.
 */
public class ArcadeDrive extends Command {
  /** Drive base being controlled. */
  final private IDrivebase m_drivebase;
  /** Supplies linear speed values. */
  final private Supplier<Double> m_speed;
  /** Supplies rotational speed values. */
  final private Supplier<Double> m_rotation;

  /**
   * Creates a new ArcadeDrive.
   *
   * @param drivebase drive base to be controlled
   * @param speed     linear speed supplier
   * @param rotation  rotational speed supplier
   */
  public ArcadeDrive(IDrivebase drivebase, Supplier<Double> speed, Supplier<Double> rotation) {
    this.m_drivebase = drivebase;
    this.m_speed = speed;
    this.m_rotation = rotation;

    addRequirements(drivebase.asSubsystem());
  }

  /**
   * Invoked to update the drivebase speeds, based on the suppliers.
   *
   * @see #initialize()
   * @see #execute()
   */
  private void updateSpeeds() {
    m_drivebase.arcadeDrive(
        IDrivebase.MAX_SPEED.times(m_speed.get()), IDrivebase.MAX_ROTATION.times(m_rotation.get()));
  }

  @Override
  public void initialize() {
    updateSpeeds();
  }

  @Override
  public void execute() {
    updateSpeeds();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
