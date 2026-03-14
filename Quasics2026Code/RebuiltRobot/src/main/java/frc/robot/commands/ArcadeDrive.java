// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.logging.Logger;
import frc.robot.logging.Logger.Verbosity;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.real.AbstractDrivebase;
import java.util.function.Supplier;

/**
 * Implements "arcade drive" support for the drivebase.
 */
public class ArcadeDrive extends Command {
  IDrivebase m_drivebase;
  private final Supplier<Double> m_linearSpeedSupplier;
  private final Supplier<Double> m_turnSpeedSupplier;
  private final Logger m_logger = new Logger(Logger.Verbosity.Info, "ArcadeDrive");

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Supplier<Double> linearSpeedSupplier,
      Supplier<Double> turnSpeedSupplier, IDrivebase drivebase) {
    m_drivebase = drivebase;
    m_linearSpeedSupplier = linearSpeedSupplier;
    m_turnSpeedSupplier = turnSpeedSupplier;
    addRequirements((Subsystem) drivebase);
  }

  /**
   * Iff true, use the inputs from the joysticks directly for speed control;
   * otherwise, scale them against maximum speeds.
   */
  private static final boolean USE_SPEED_INPUTS_DIRECTLY = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double linearSpeedPercent = MathUtil.clamp(m_linearSpeedSupplier.get(), -1.0, +1.0);
    final double angularVelocityPercent = MathUtil.clamp(m_turnSpeedSupplier.get(), -1.0, +1.0);

    if (USE_SPEED_INPUTS_DIRECTLY) {
      m_logger.log(
          String.format(
              "linearSpeedPercent = %0.3f, angularVelocityPercent = %0.3f",
              linearSpeedPercent, angularVelocityPercent),
          Verbosity.Debug);
      m_drivebase.arcadeDrive(linearSpeedPercent, angularVelocityPercent);
    } else {
      // Taking the values from the speed suppliers as a "% of top speed", and
      // then multiply them by the top speeds that the team is comfortable setting
      // for movement/turning. This lets us define maximum speeds someplace safely
      // (e.g., in AbstractDrivebase), and then have the code work in terms of
      // those values.
      //
      // TODO: Note that this isn't actually what's *happening* in the current
      // implementation of the drivebase code. (See comments there.)
      final LinearVelocity linearSpeed = AbstractDrivebase.getMaxMotorLinearSpeed().times(linearSpeedPercent);
      final AngularVelocity angularVelocity = AbstractDrivebase.getMaxMotorTurnSpeed().times(angularVelocityPercent);

      m_logger.log(
          String.format(
              "linearSpeedPercent = %0.3f, angularVelocityPercent = %0.3f, linearSpeed = %s, angularVelocity = ",
              linearSpeedPercent, angularVelocityPercent, linearSpeed.toShortString(), angularVelocity.toShortString()),
          Verbosity.Debug);
      m_drivebase.arcadeDrive_speeds(linearSpeed, angularVelocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
