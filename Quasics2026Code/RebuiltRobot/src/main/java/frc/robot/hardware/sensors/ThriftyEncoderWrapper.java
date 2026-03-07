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

/*
 * Sample data from DriveForDistance
 * 
 * Starting driving at 0.25 power, from 9.636e-05 m to 2.000e+00 m
 * teleopPeriodic(): 0.000556s
 * SmartDashboard.updateValues(): 0.069470s
 * robotPeriodic(): 0.001688s
 * LiveWindow.updateValues(): 0.000000s
 * Shuffleboard.update(): 0.000018s
 * Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:62):
 * teleopPeriodic(): 0.000556s
 * SmartDashboard.updateValues(): 0.069470s
 * robotPeriodic(): 0.001688s
 * LiveWindow.updateValues(): 0.000000s
 * Shuffleboard.update(): 0.000018s
 * Reported left distance: 0.0008 m (delta: 0.0007 m, raw: 0.5952 units),
 * velocity: 0.0002 m/s (sampled: 0.01)
 * Forward - isFinished --> false
 * Reported left distance: 0.0011 m (delta: 0.0003 m, raw: 0.8095 units),
 * velocity: 0.0002 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0011 m (delta: 0.0000 m, raw: 0.8095 units),
 * velocity: 0.0002 m/s (sampled: 0.00)
 * Forward - isFinished --> false
 * Reported left distance: 0.0014 m (delta: 0.0003 m, raw: 1.0476 units),
 * velocity: 0.0002 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0017 m (delta: 0.0003 m, raw: 1.2857 units),
 * velocity: 0.0002 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0021 m (delta: 0.0004 m, raw: 1.5476 units),
 * velocity: 0.0002 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0024 m (delta: 0.0003 m, raw: 1.7857 units),
 * velocity: 0.0002 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0028 m (delta: 0.0004 m, raw: 2.0476 units),
 * velocity: 0.0003 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0032 m (delta: 0.0004 m, raw: 2.3571 units),
 * velocity: 0.0003 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0036 m (delta: 0.0004 m, raw: 2.6667 units),
 * velocity: 0.0003 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0041 m (delta: 0.0005 m, raw: 3.0238 units),
 * velocity: 0.0003 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0046 m (delta: 0.0005 m, raw: 3.4048 units),
 * velocity: 0.0004 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0051 m (delta: 0.0005 m, raw: 3.7857 units),
 * velocity: 0.0004 m/s (sampled: 0.02)
 * Forward - isFinished --> false
 * Reported left distance: 0.0056 m (delta: 0.0005 m, raw: 4.1667 units),
 * velocity: 0.0004 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0062 m (delta: 0.0005 m, raw: 4.5714 units),
 * velocity: 0.0004 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0062 m (delta: 0.0000 m, raw: 4.5714 units),
 * velocity: 0.0004 m/s (sampled: 0.00)
 * Forward - isFinished --> false
 * Reported left distance: 0.0067 m (delta: 0.0006 m, raw: 5.0000 units),
 * velocity: 0.0004 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0073 m (delta: 0.0006 m, raw: 5.4286 units),
 * velocity: 0.0004 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0079 m (delta: 0.0006 m, raw: 5.8810 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0085 m (delta: 0.0006 m, raw: 6.3333 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0092 m (delta: 0.0006 m, raw: 6.7857 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0097 m (delta: 0.0006 m, raw: 7.2143 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0104 m (delta: 0.0006 m, raw: 7.6905 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0110 m (delta: 0.0006 m, raw: 8.1667 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0117 m (delta: 0.0006 m, raw: 8.6429 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0123 m (delta: 0.0007 m, raw: 9.1429 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0130 m (delta: 0.0006 m, raw: 9.6190 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0136 m (delta: 0.0006 m, raw: 10.0952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0143 m (delta: 0.0007 m, raw: 10.5952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0150 m (delta: 0.0007 m, raw: 11.0952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0156 m (delta: 0.0007 m, raw: 11.5952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0163 m (delta: 0.0007 m, raw: 12.0952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0170 m (delta: 0.0007 m, raw: 12.5952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0177 m (delta: 0.0007 m, raw: 13.0952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0177 m (delta: 0.0000 m, raw: 13.0952 units),
 * velocity: 0.0005 m/s (sampled: 0.00)
 * Forward - isFinished --> false
 * Reported left distance: 0.0184 m (delta: 0.0007 m, raw: 13.6190 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0190 m (delta: 0.0006 m, raw: 14.0952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0197 m (delta: 0.0007 m, raw: 14.5952 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0204 m (delta: 0.0007 m, raw: 15.1190 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0211 m (delta: 0.0007 m, raw: 15.6190 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0218 m (delta: 0.0007 m, raw: 16.1667 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0225 m (delta: 0.0006 m, raw: 16.6429 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0232 m (delta: 0.0007 m, raw: 17.1667 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0238 m (delta: 0.0007 m, raw: 17.6667 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0245 m (delta: 0.0007 m, raw: 18.1905 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0252 m (delta: 0.0007 m, raw: 18.7143 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0259 m (delta: 0.0006 m, raw: 19.1905 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0266 m (delta: 0.0007 m, raw: 19.6905 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0272 m (delta: 0.0007 m, raw: 20.1905 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0279 m (delta: 0.0007 m, raw: 20.7143 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0287 m (delta: 0.0007 m, raw: 21.2381 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0293 m (delta: 0.0007 m, raw: 21.7381 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0300 m (delta: 0.0007 m, raw: 22.2381 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0307 m (delta: 0.0007 m, raw: 22.7381 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0313 m (delta: 0.0007 m, raw: 23.2381 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0313 m (delta: 0.0000 m, raw: 23.2381 units),
 * velocity: 0.0005 m/s (sampled: 0.00)
 * Forward - isFinished --> false
 * Reported left distance: 0.0321 m (delta: 0.0007 m, raw: 23.7619 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0327 m (delta: 0.0007 m, raw: 24.2619 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0334 m (delta: 0.0007 m, raw: 24.7857 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0341 m (delta: 0.0006 m, raw: 25.2619 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0348 m (delta: 0.0007 m, raw: 25.8095 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0355 m (delta: 0.0006 m, raw: 26.2857 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0362 m (delta: 0.0007 m, raw: 26.8095 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0368 m (delta: 0.0007 m, raw: 27.3095 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0375 m (delta: 0.0007 m, raw: 27.8095 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0383 m (delta: 0.0007 m, raw: 28.3571 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0389 m (delta: 0.0006 m, raw: 28.8333 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0396 m (delta: 0.0007 m, raw: 29.3333 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0403 m (delta: 0.0007 m, raw: 29.8571 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0410 m (delta: 0.0007 m, raw: 30.3571 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0417 m (delta: 0.0007 m, raw: 30.8810 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0423 m (delta: 0.0007 m, raw: 31.3810 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0430 m (delta: 0.0007 m, raw: 31.9048 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * Reported left distance: 0.0430 m (delta: 0.0000 m, raw: 31.9048 units),
 * velocity: 0.0005 m/s (sampled: 0.00)
 * Forward - isFinished --> false
 * Reported left distance: 0.0437 m (delta: 0.0007 m, raw: 32.4048 units),
 * velocity: 0.0005 m/s (sampled: 0.03)
 * Forward - isFinished --> false
 * Reported left distance: 0.0445 m (delta: 0.0007 m, raw: 32.9524 units),
 * velocity: 0.0005 m/s (sampled: 0.04)
 * Forward - isFinished --> false
 * 
 */