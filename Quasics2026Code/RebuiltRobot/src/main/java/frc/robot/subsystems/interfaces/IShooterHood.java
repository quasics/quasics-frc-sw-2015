// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Defines an interface for working with a positionable shooter hood.
 *
 * Note that while we may not need to use PID control for the hood, doing so
 * (and setting up the motor controller to use closed-loop control via the
 * absolute encoder with the through-bore encoder wired to it) would be a
 * good idea to ensure that the hood is always at the correct angle. And this
 * would mean you'd need to do comparatively little work to implement it,
 * since the SparkMax controllers have built-in support for closed-loop
 * control with an absolute encoder (and the through-bore encoder can be wired
 * to the SparkMax's encoder port, so you wouldn't need to do any custom
 * wiring or anything like that).
 *
 * And, yeah: this would also allow us to easily set the hood to specific
 * angles for different shots, which could be useful for consistency and
 * accuracy.
 *
 * @see https://docs.revrobotics.com/brushless/spark-max/encoders
 * @see <a href="https://shorturl.at/wm3tF">Through Bore encoder application
 *      notes</a>
 * @see <a href=
 *      "https://docs.revrobotics.com/revlib/spark/closed-loop">Closed-loop
 *      control with SparkMax</a>
 */
public interface IShooterHood {
  /**
   * Returns the hood's current angle.
   */
  // FINDME(Daniel): This should return a WPILib "Angle" object (from the Units
  // library), rather than a "naked value" (which could be degrees, radians,
  // etc.).
  //
  // Note: I've fixed this for you, but it's a habit worth building....
  Angle getCurrentAngle();

  /**
   * Starts the hood moving down (under manual control) at the specified speed.
   * 
   * @param speed
   */
  void moveDown(double speed);

  /**
   * Starts the hood moving up (under manual control) at the specified speed.
   * 
   * @param speed
   */
  void moveUp(double speed);

  /** Stops the hood's motion. */
  void stop();

  /**
   * Null implementation of IShooterHood, for use on robots that don't have a
   * hood.
   */
  public class NullShooterHood extends SubsystemBase implements IShooterHood {
    @Override
    public Angle getCurrentAngle() {
      return Degrees.of(0);
    }

    @Override
    public void moveDown(double speed) {
      // No-op.
    }

    @Override
    public void moveUp(double speed) {
      // No-op.
    }

    @Override
    public void stop() {
      // No-op.
    }
  }
}
