// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.util.Conversion;
import com.thethriftybot.util.Conversion.PositionUnit;
import com.thethriftybot.util.Conversion.VelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;

import java.io.IOException;

/**
 * Convenience wrapper, allowing a ThriftyNova to be read in the same (general)
 * way as a normal WPLib Encoder.
 */
public class ThriftyEncoderWrapper implements TrivialEncoder {
  /**
   * Thrifty conversion object, used to translate native velocity units to
   * rotations/sec.
   */
  final static Conversion m_speedConverter = new Conversion(VelocityUnit.ROTATIONS_PER_SEC, EncoderType.INTERNAL);

  /**
   * Thrifty conversion object, used to translate native positional units to
   * rotations.
   */
  final static Conversion m_distanceConverter = new Conversion(PositionUnit.ROTATIONS, EncoderType.INTERNAL);

  /** Wrapped Thrifty Nova controller, providing access to encoder data. */
  final ThriftyNova m_motorController;

  /**
   * Outer diameter of the wheel. (Used to convert "revolutions" to linear
   * distance.)
   */
  final Distance m_wheelCircumference;

  final double m_gearing;
  final double m_distanceScalingFactorForGearing;
  final double m_velocityScalingFactor;

  /**
   * Constructor. (This assumes that the motor is directly connected to the wheel,
   * or effectively having 1:1 gearing).
   *
   * @param motorController    ThriftyNova object being wrapped for "normal" use
   * @param wheelOuterDiameter outer diameter of the wheel being turned by the
   *                           motor
   */
  public ThriftyEncoderWrapper(
      ThriftyNova motorController, Distance wheelOuterDiameter) {
    this(motorController, wheelOuterDiameter, 1.0);
  }

  static final boolean USE_SPARK_CALCULATIONS = true;

  /**
   * Constructor.
   *
   * @param motorController    ThriftyNova object being wrapped for "normal" use
   * @param wheelOuterDiameter outer diameter of the wheel being turned by the
   *                           motor
   * @param gearing            gearing ratio from the motor to the wheel (e.g.,
   *                           8.45)
   */
  public ThriftyEncoderWrapper(
      ThriftyNova motorController, Distance wheelOuterDiameter, double gearing) {
    m_motorController = motorController;
    m_wheelCircumference = wheelOuterDiameter.times(Math.PI);
    m_gearing = gearing;

    final Distance wheelCircumference = wheelOuterDiameter.times(Math.PI);
    m_distanceScalingFactorForGearing = wheelCircumference.div(Constants.drivebaseGearRatio).in(Meters);
    m_velocityScalingFactor = m_distanceScalingFactorForGearing / 60;
    if (USE_SPARK_CALCULATIONS) {
      System.out.println("Wheel circumference: " + wheelCircumference);
      System.out.println("Using gear ratio: " + Constants.drivebaseGearRatio);
      System.out.println("Adjustment for gearing (m/rotation): " + m_distanceScalingFactorForGearing);
      System.out.println("Velocity adj.: " + m_velocityScalingFactor);
    }
  }

  @Override
  public Distance getPosition() {
    /*
     * Math for SparkMax controller setup:
     * 
     * final Distance wheelCircumference = Constants.wheelRadius.times(2 * Math.PI);
     * 
     * final double scalingFactor_geared =
     * wheelCircumference.div(Constants.drivebaseGearRatio).in(Meters);
     * 
     * final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
     * System.out.println("Wheel circumference: " + wheelCircumference);
     * System.out.println("Using gear ratio: " + Constants.drivebaseGearRatio);
     * System.out.println("Adjustment for gearing (m/rotation): " +
     * distanceScalingFactorForGearing);
     * System.out.println("Velocity adj.: " + velocityScalingFactor);
     */
    final double currentRevolutions = m_distanceConverter.fromMotor(m_motorController.getPosition());
    if (USE_SPARK_CALCULATIONS) {
      return Meters.of(currentRevolutions * m_distanceScalingFactorForGearing);
    } else {
      // revolutions * circumferenceTraveled/revolution / gearingRatio --> distance
      return m_wheelCircumference.times(currentRevolutions / m_gearing);
    }
  }

  @Override
  public LinearVelocity getVelocity() {
    final double currentRotationsPerSecond = m_speedConverter.fromMotor(m_motorController.getVelocity());
    if (USE_SPARK_CALCULATIONS) {
      return MetersPerSecond.of(currentRotationsPerSecond * m_velocityScalingFactor);
    } else {
      // (revs/sec) * circumferenceTraveled/rev / gearingRatio) --> (meters/sec)
      final double metersPerRotation = m_wheelCircumference.in(Meters);
      return MetersPerSecond.of(currentRotationsPerSecond * metersPerRotation / m_gearing);
    }
  }

  @Override
  public void reset() {
    m_motorController.setEncoderPosition(0);
  }

  @Override
  public void close() throws IOException {
    // No-op: ThriftyNova should be closed through the MotorController
    // interface.
  }

  @Override
  public double getRawPosition() {
    return m_motorController.getPosition();
  }
}
