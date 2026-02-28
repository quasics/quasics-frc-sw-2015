// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface IShooterHood {
  /*
   * (1 motor for hood angle; rotary (absolute) encoder for hood position)
   * Adjust hood angle (with limits) - forward and backward
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
   * FINDME(Daniel): Some suggested reading for this subsystem:
   * * SparkMax controllers with different encoders:
   * https://docs.revrobotics.com/brushless/spark-max/encoders
   * * Through Bore encoder application notes: https://shorturl.at/wm3tF
   * * Closed-loop control with SparkMax:
   * https://docs.revrobotics.com/revlib/spark/closed-loop
   *
   */

  // FINDME(Daniel): This should return a WPILib "Angle" object (from the Units
  // library), rather than a "naked value" (which could be degrees, radians,
  // etc.).
  double getCurrentAngle();

  void moveOut(double speed);

  void moveIn(double speed);

  void stop();

  /**
   * Null implementation of IShooterHood, for use on robots that don't have a
   * hood.
   */
  public class NullShooterHood extends SubsystemBase implements IShooterHood {
    @Override
    public double getCurrentAngle() {
      return 0;
    }

    @Override
    public void moveOut(double speed) {
      // No-op.
    }

    @Override
    public void moveIn(double speed) {
      // No-op.
    }

    @Override
    public void stop() {
      // No-op.
    }
  }
}
