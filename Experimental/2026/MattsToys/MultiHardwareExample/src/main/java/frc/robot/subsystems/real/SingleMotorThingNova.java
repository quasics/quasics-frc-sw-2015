// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

// import com.thethriftybot.ThriftyNova;
import frc.robot.subsystems.implementation.SingleMotorThing;

public class SingleMotorThingNova extends SingleMotorThing {
  /**
   * Builds the actual hardware wrappers that will be passed to the base class.
   */
  static DerivedClassData getStuffForBaseClassSetup() {
    // ThriftyNova motorController = new ThriftyNova(1, ThriftyNova.MotorType.NEO);
    // // Velocity reading examples
    // Conversion shooterConverter = new Conversion(VelocityUnit.ROTATIONS_PER_MIN,
    // EncoderType.INTERNAL); double currentRPM = shooterConverter.fromMotor(motor.getVelocity());

    // return new DerivedClassData(motorController, encoder);
    return null;
  }

  public SingleMotorThingNova() {
    super(getStuffForBaseClassSetup());
  }
}
