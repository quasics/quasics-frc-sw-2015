// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Sample interface for a shooter hood subsystem, which controls the angle of
 * the shooter hood to adjust the trajectory of the ball. The hood is typically
 * controlled by a motor with an absolute encoder for feedback, and uses a PID
 * controller to move to the desired angle.
 */
public interface IShooterHood extends ISubsystem {
  /** Name of the subsystem. */
  final static String SUBSYSTEM_NAME = "ShooterHood";

  /**
   * Sets the target position for the shooter hood. The hood will then move to the
   * target position under PID control.
   * 
   * @param targetAngle target angle for the shooter hood; this will be clamped to
   *                    the range [kMinPosDegrees, kMaxPosDegrees] to prevent
   *                    mechanical issues.
   */
  void setPosition(Angle targetAngle);

  /** Returns the current position of the shooter hood. */
  Angle getCurrentPosition();

  /** Returns the target position of the shooter hood. */
  Angle getTargetAngle();

  /** Returns the minimum angle for the shooter hood. */
  Angle getMinAngle();

  /** Returns the maximum angle for the shooter hood. */
  Angle getMaxAngle();

  /**
   * Null implementation of the IShooterHood interface, for use (if needed) when
   * the subsystem is not present.
   */
  public class NullShooterHood extends SubsystemBase implements IShooterHood {
    public NullShooterHood() {
      setName(SUBSYSTEM_NAME);
    }

    @Override
    public void setPosition(Angle targetAngle) {
      // No-op
    }

    @Override
    public Angle getCurrentPosition() {
      return Degrees.of(0);
    }

    @Override
    public Angle getTargetAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getMinAngle() {
      return Degrees.of(0);
    }

    @Override
    public Angle getMaxAngle() {
      return Degrees.of(0);
    }

    @Override
    public void close() throws IOException {
      // No-op
    }
  }
}
