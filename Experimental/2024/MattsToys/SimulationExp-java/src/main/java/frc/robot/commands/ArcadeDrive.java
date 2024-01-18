// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AbstractDrivebase;

/**
 * Command allowing users to drive the robot using "arcade-drive" mechanism,
 * with separate inputs independently controlling the forward/backward and
 * rotational speeds. (Note that these could be coming from a single joystick's
 * X/Y axes, for traditional arcade drive, or 2 separate joysticks for "split"
 * arcade drive.)
 */
public class ArcadeDrive extends Command {
  private final AbstractDrivebase m_drivebase;
  private final Supplier<Double> m_xSupplier;
  private final Supplier<Double> m_rotSupplier;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(AbstractDrivebase drivebase, Supplier<Double> xSupplier, Supplier<Double> rotSupplier) {
    m_xSupplier = xSupplier;
    m_rotSupplier = rotSupplier;
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
    final double xInput = m_xSupplier.get();
    final double rotInput = m_rotSupplier.get();

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    Measure<Velocity<Distance>> xSpeed = AbstractDrivebase.MAX_SPEED.times(-m_speedLimiter.calculate(xInput));

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    Measure<Velocity<Angle>> rot = AbstractDrivebase.MAX_ANGULAR_SPEED.times(-m_rotLimiter.calculate(rotInput));

    m_drivebase.arcadeDrive(xSpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
