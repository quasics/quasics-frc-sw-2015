// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.hardware.actuators.IMotorControllerPlus;
import frc.robot.hardware.actuators.TalonMotorControllerPlus;
import frc.robot.hardware.sensors.TalonEncoderWrapper;
import frc.robot.hardware.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;
import frc.robot.subsystems.implementation.SingleMotorThingGroup;
import frc.robot.subsystems.interfaces.ISingleMotorThing;
import frc.robot.subsystems.real.SingleMotorThingNova;
import frc.robot.subsystems.real.SingleMotorThingSpark;
import frc.robot.subsystems.real.SingleMotorThingTalonPwm;
import frc.robot.subsystems.simulation.SingleMotorThingSim;

public class SmtStaticAllocator {

  public static final int RIGHT_INTAKE_DEPLOYMENT_ID = 6;
  public static final int LEFT_INTAKE_DEPLOYMENT_ID = 7;

  /** Supported (fixed) hardware configurations. */
  public enum HardwareConfig {
    /** Simulated motor only. */
    Simulated,
    /** SparkMax motor controller (live). */
    Spark,
    /** Paired SparkMax motor controllers (live). */
    SparkPair,
    /** TalonFX motor controller (live). */
    TalonPwm,
    /**
     * TalonFX motor controller, constructed directly from the basic
     * SingleMotorThing class (live).
     */
    TalonCanDirect,
    /** Thrifty Nova motor controller (live). */
    Thrifty,
    /** Victor motor controller (live). */
    Victor,
  }

  /**
   * Demonstrates how to set up a "single motor thing" in a number of ways, based
   * on the selected hardware configuration.
   * 
   * @param HardwareConfig the (fixed/template) hardware configuration to use
   */
  static public ISingleMotorThing allocateFixedSingleMotorThing(HardwareConfig config) {
    return switch (config) {
      case Simulated -> new SingleMotorThingSim();
      case Spark -> new SingleMotorThingSpark(5, true);
      case SparkPair -> {
        // Generate an example of a pair of motors being treated as a single
        // (logical) entity (e.g., paired motors responsible for
        // extending/retracting an intake, mounted in opposite directions from
        // each other).
        yield new SingleMotorThingGroup(
            new SingleMotorThingSpark(RIGHT_INTAKE_DEPLOYMENT_ID, true),
            new SingleMotorThingSpark(LEFT_INTAKE_DEPLOYMENT_ID, false));
      }
      case TalonPwm -> new SingleMotorThingTalonPwm(6);
      case TalonCanDirect -> {
        final TalonFX talon = new TalonFX(11);
        final IMotorControllerPlus motorController = new TalonMotorControllerPlus(talon);
        final TrivialEncoder encoder = new TalonEncoderWrapper(talon, Inches.of(6));
        yield new SingleMotorThing(
            new SingleMotorThing.ConstructionData(motorController, encoder));
      }
      case Thrifty -> new SingleMotorThingNova(5);
      case Victor ->
        // Sample of how to use the SingleMotorThing class without needing
        // to derive a class for hardware-specific setup.
        new SingleMotorThing(new SingleMotorThing.ConstructionData(
            // Use a Victor motor controller...
            new VictorSP(8),
            // ...and a bog-standard WPILib encoder.
            TrivialEncoder.forWpiLibEncoder(new Encoder(1, 2))));
    };
  }

}
