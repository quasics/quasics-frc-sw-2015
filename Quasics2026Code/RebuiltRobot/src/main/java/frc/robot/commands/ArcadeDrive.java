// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Supplier<Double> linearSpeedSupplier,
      Supplier<Double> turnSpeedSupplier, IDrivebase drivebase) {
    m_drivebase = drivebase;
    m_linearSpeedSupplier = linearSpeedSupplier;
    m_turnSpeedSupplier = turnSpeedSupplier;
    addRequirements((Subsystem) drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  static final boolean LOG_DATA = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Taking the values from the speed suppliers as a "% of top speed", and
    // then multiply them by the top speeds that the team is comfortable setting
    // for movement/turning. This lets us define maximum speeds someplace safely
    // (e.g., in AbstractDrivebase), and then have the code work in terms of
    // those values.

    // FINDME(Robert): This also let us (easily) write code that checks the
    // values, to make sure that someone doesn't specify something that's
    // "overspeed" for the robot (e.g., the %age #s should always be between
    // -1.0 and +1.0, and then you just use stuff like MathUtils.clamp() to
    // ensure that the values are in that range).

    final double linearSpeedPercent = m_linearSpeedSupplier.get();
    final LinearVelocity linearSpeed = AbstractDrivebase.getMaxMotorLinearSpeed().times(linearSpeedPercent);
    final double angularVelocityPercent = m_turnSpeedSupplier.get();
    final AngularVelocity angularVelocity = AbstractDrivebase.getMaxMotorTurnSpeed().times(angularVelocityPercent);

    if (LOG_DATA) {
      System.out.println("turnSpeedSupplier = " + linearSpeed);
      System.out.println("angularVelocity = " + angularVelocity);
    }
    m_drivebase.arcadeDrive(linearSpeed, angularVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
