// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;

/** Support functions for working with REV hardware. */
public class RevSupportFunctions {
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
   * Configures the SparkMax for default (revolutions/RPM-based) reporting of
   * encoder data.
   *
   * @param sparkMaxConfig the configuration to update
   */
  public static void configureForRpm(SparkMaxConfig sparkMaxConfig) {
    sparkMaxConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
  }

  /**
   * Updates a SparkMaxConfig to report radian-based values (radians/sec) for
   * distance/velocity, rather than the native units (rotations and RPM).
   *
   * @param sparkMaxConfig the configuration to update
   */
  public static void configureForRadians(SparkMaxConfig sparkMaxConfig) {
    sparkMaxConfig.encoder.positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60);
  }

  /**
   * Updates a SparkMaxConfig to report radian-based values (radians/sec) for
   * distance/velocity, rather than the native units (rotations and RPM).
   *
   * @param config the configuration to update
   */
  public static void configureForRadians(AbsoluteEncoderConfig config) {
    config.positionConversionFactor(2 * Math.PI).velocityConversionFactor(2 * Math.PI / 60);
  }
}
