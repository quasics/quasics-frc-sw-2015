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
 * Demonstrates how a base class can be set up to handle just about everything for a subsystem,
 * except for the specific setup of hardware.
 */
public class SingleMotorThing extends SubsystemBase implements ISingleMotorThing {
  final MotorController controller;
  final TrivialEncoder encoder;

  /**
   * Defines the set of data needed for constructing this class (from derived classes).
   *
   * This is defined a a single unit, so that if these things are "connected" in subclasses (e.g.,
   * need to ask the controller for its encoder), we can do that easily.
   *
   * @see frc.robot.subsystems.real.SingleMotorThingPwmSpark for an example
   */
  public record DerivedClassData(MotorController controller, TrivialEncoder encoder) {}

  /**
   * Creates a new AbstractSingleMotorThing.
   *
   * This would be called from the derived class.
   */
  protected SingleMotorThing(DerivedClassData data) {
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
