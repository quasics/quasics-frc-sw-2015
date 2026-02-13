package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

/**
 * Sample interface for a shooter hood subsystem, which controls the angle of
 * the shooter hood to adjust the trajectory of the ball. The hood is typically
 * controlled by a motor with an absolute encoder for feedback, and uses a PID
 * controller to move to the desired angle.
 * 
 * TODO: Update min/max angles based on the actual mechanism design and testing.
 */
public interface IShooterHood extends ISubsystem {
  static final Angle kMinPos = Degrees.of(15.0);
  static final Angle kMaxPos = Degrees.of(85.0);

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
}
