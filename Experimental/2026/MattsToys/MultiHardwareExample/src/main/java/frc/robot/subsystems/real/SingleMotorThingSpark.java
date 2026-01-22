// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;

/**
 * Provides a simple example of a "SingleMotorThing" that uses Spark
 * controllers.
 *
 * Note that all of the "actual functionality" takes place in the base class;
 * this class only exists
 * to set up the hardware-specific stuff.
 */
public class SingleMotorThingSpark extends SingleMotorThing {
  /** Encoder ticks per revolution. */
  public static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /**
   * Gearing ratio used to drive the actual wheel to which we assume we're
   * attached. (This is used to configure the encoder.)
   */
  public static final double GEAR_RATIO = 8.45;

  /** Wheel diameter in inches. */
  public static final Distance WHEEL_DIAMETER = Inches.of(6);

  /**
   * Updates a SparkMaxConfig to work with distance-based values (meters and
   * meters/sec), rather than the native rotation-based units (rotations and RPM).
   *
   * @param config        the object being configured
   * @param outerDiameter distance of the object (wheel, sprocket, etc.) being
   *                      turned
   * @param gearRatio     gearing ratio of the motor to the object being turned
   *                      (i.e., given a ratio of 1 turn of the external object
   *                      for every N turns of the motor, this would be N)
   */
  public static void configureSparkMaxEncoderForDistance(
      SparkMaxConfig config, Distance outerDiameter, double gearRatio) {
    final double distanceScalingFactorForGearing = outerDiameter.div(gearRatio).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    config.encoder.positionConversionFactor(distanceScalingFactorForGearing)
        .velocityConversionFactor(velocityScalingFactor);
  }

  /**
   * Builds the actual hardware wrappers that will be passed to the base class.
   */
  static DerivedClassData getStuffForBaseClassSetup() {
    SparkMax motorController = new SparkMax(1, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    configureSparkMaxEncoderForDistance(config, WHEEL_DIAMETER, GEAR_RATIO);
    motorController.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    TrivialEncoder encoder = new SparkMaxEncoderWrapper(motorController.getAlternateEncoder());

    return new DerivedClassData(motorController, encoder);
  }

  /** Creates a new SingleMotorThingSpark. */
  public SingleMotorThingSpark() {
    super(getStuffForBaseClassSetup());
  }
}
