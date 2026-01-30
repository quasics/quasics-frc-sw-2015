// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.implementation;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.interfaces.ISingleMotorThing;

/**
 * Demonstrates how a base class can be set up to handle just about everything
 * for a subsystem, except for the *specific* setup of hardware (generally to be
 * handled by a "leaf" derived class).
 *
 * @see frc.robot.subsystems.real.SingleMotorThingSpark
 * @see frc.robot.subsystems.real.SingleMotorThingTalon
 */
public class SingleMotorThing extends SubsystemBase implements ISingleMotorThing {
  /**
   * Defines the set of data needed for constructing this class (from derived
   * classes).
   *
   * This is defined a a single unit, so that if these things are "connected" in
   * subclasses (e.g., need to ask the controller for its encoder), we can do that
   * easily.
   *
   * Note: despite the name, this isn't *really* just for derived classes (which
   * is why this is public, rather than protected). (There's an example in the
   * RobotContainer of how a SingleMotorThing could be built directly by a
   * client.)
   *
   * @see frc.robot.subsystems.real.SingleMotorThingPwmSpark for an example
   */
  public record DerivedClassData(MotorController controller, TrivialEncoder encoder) {
  }

  /**
   * The motor controller used for this "thing". This will be allocated by the
   * "leaf" derived class.
   */
  final MotorController controller;

  /**
   * The encoder used for this "thing". This will be allocated by the "leaf"
   * derived class.
   *
   * Note that I'm using the "TrivialEncoder" wrapper that I put together some
   * years back. This is because there isn't a WPILib-standard interface (or base
   * class) used for all encoders, and thus this lets us handle any sort of an
   * encoder as a common type in code.
   */
  final TrivialEncoder encoder;

  /**
   * Creates a new SingleMotorThing.
   *
   * This would usually be called from the derived class, but you could
   * theoretically do this directly from a client that builds its own
   * DerivedClassData record.
   */
  public SingleMotorThing(DerivedClassData data) {
    this.controller = data.controller;
    this.encoder = data.encoder;
  }

  @Override
  public void setSpeed(double percent) {
    controller.set(percent);
  }

  @Override
  public double getSpeed() {
    return controller.get();
  }

  @Override
  public Distance getPosition() {
    return encoder.getPosition();
  }
}
