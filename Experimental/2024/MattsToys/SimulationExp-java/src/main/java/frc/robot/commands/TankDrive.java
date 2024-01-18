// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

/**
 * Command allowing users to drive the robot using "tank-drive" mechanism, with
 * separate inputs independently controlling the left and right wheel speeds.
 */
public class TankDrive extends Command {
  private final AbstractDrivebase m_drivebase;
  private final Supplier<Double> m_leftSupplier;
  private final Supplier<Double> m_rightSupplier;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(3);

  /** Creates a new TankDrive. */
  public TankDrive(AbstractDrivebase drivebase, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    m_leftSupplier = leftSupplier;
    m_rightSupplier = rightSupplier;
    m_drivebase = drivebase;

    addRequirements(drivebase);
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

  /**
   * Actually reads the controllers, calculates the speeds we want, and tells the
   * drive base to "make it so".
   */
  private void updateSpeeds() {
    final double leftInput = m_leftSupplier.get();
    final double rightInput = m_rightSupplier.get();

    final Measure<Velocity<Distance>> leftSpeed = AbstractDrivebase.MAX_SPEED
        .times(m_leftSpeedLimiter.calculate(leftInput));
    final Measure<Velocity<Distance>> rightSpeed = AbstractDrivebase.MAX_SPEED
        .times(m_rightSpeedLimiter.calculate(rightInput));

    m_drivebase.setSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
