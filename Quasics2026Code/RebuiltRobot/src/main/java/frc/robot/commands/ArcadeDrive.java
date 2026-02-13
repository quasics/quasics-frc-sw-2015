// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.real.AbstractDrivebase;

import java.util.function.Supplier;

/**
 * Implements "arcade drive" support for the drivebase.
 */
public class ArcadeDrive extends Command {
  AbstractDrivebase m_drivebase;
  private final Supplier<Double> m_linearSpeedSupplier;
  private final Supplier<Double> m_turnSpeedSupplier;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Supplier<Double> linearSpeedSupplier,
      Supplier<Double> turnSpeedSupplier, AbstractDrivebase drivebase) {
    m_drivebase = drivebase;
    m_linearSpeedSupplier = linearSpeedSupplier;
    m_turnSpeedSupplier = turnSpeedSupplier;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // FINDME(Robert): I don't think this is what you want to do. If the values
    // are coming straight from the joystick, then the robot will never be able
    // to move more than 1 m/s (or turn faster than 1 radian/sec), since the
    // joysticks only provide values from -1.0 to +1.0.
    //
    // Suggestion: one option would be to take the values from the speed
    // suppliers as a "% of top speed", and then multiply them by the top speeds
    // that the team is comfortable setting for movement/turning, in the same
    // way that you're doing for the LinearSpeedCommand. This will let you define
    // maximum speeds someplace safely (e.g., in AbstractDrivebase), and then have
    // the code work in terms of those values. It will also let you (easily) write
    // code that checks the values, to make sure that someone doesn't specify
    // something that's "overspeed" for the robot (e.g., the %age #s should always
    // be between -1.0 and +1.0, and then you just use stuff like )

    // LinearVelocity linearSpeed = AbstractDrivebase.getMaxMotorSpeed() *
    // MetersPerSecond.of(m_linearSpeedSupplier.get());
    LinearVelocity linearSpeed = MetersPerSecond.of(m_linearSpeedSupplier.get());
    AngularVelocity angularVelocity = RadiansPerSecond.of(m_turnSpeedSupplier.get());
    System.out.println("turnSpeedSupplier = " + m_turnSpeedSupplier);
    System.out.println("angularVelocity = " + angularVelocity);
    m_drivebase.arcadeDrive(linearSpeed, angularVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: Implement and Call m_drivebase.stop()
    // FINDME(Robert): I've done this for you, because it's important. (And
    // because what to do was already on the line above. :-) -mjh
    m_drivebase.stop();
  }
}
