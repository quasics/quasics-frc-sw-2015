// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.RevSupportFunctions.configureAsNotFollowing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IShooterHood;
import frc.robot.util.config.HoodConfig;

import java.io.IOException;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

/**
 * Sample implementation of the IShooterHood subsystem. This subsystem controls
 * the angle of the shooter hood using a motor with an absolute encoder for
 * feedback, and uses a PID controller to move to the desired angle.
 */
public class ShooterHoodSubsystem extends SubsystemBase implements IShooterHood {
  /**
   * PID settings for the shooter hood.
   * Note that the PID constants (kP, kI, kD) would need to be tuned for the
   * actual mechanism; these are just starting points.
   *
   * Note also that the P-gain is adjusted for degree units (would be more like
   * 0.1 if we were using rotations).
   */
  protected static final double kP = 0.05, kI = 0, kD = 0;

  /** Hood configuration data. */
  protected final HoodConfig m_config;

  /** Motor controller used to adjust the hood position. */
  protected final SparkMax m_motor;

  /** Absolute encoder used to read the current hood position. */
  protected final SparkAbsoluteEncoder m_absoluteEncoder;

  /** PID controller handling hood positioning. */
  protected final SparkClosedLoopController m_pidController;

  /**
   * Constructor.
   * 
   * @param hoodConfig configuration for the shooter hood subsystem, which
   *                   includes the control type, CAN ID, and min/max angles. The
   *                   control type is expected to be SparkMax for this
   *                   implementation, and the CAN ID should match the actual
   *                   wiring of the robot. The min/max angles will be used to
   *                   clamp the target position to prevent mechanical issues.
   */
  public ShooterHoodSubsystem(HoodConfig hoodConfig) {
    setName(SUBSYSTEM_NAME);

    if (hoodConfig.type() != HoodConfig.ControlType.SparkMax) {
      throw new IllegalArgumentException(
          "Unsupported control type for ShooterHoodSubsystem: " + hoodConfig.type());
    }

    m_config = hoodConfig;
    m_motor = new SparkMax(hoodConfig.canId(), MotorType.kBrushless);
    m_absoluteEncoder = m_motor.getAbsoluteEncoder();
    m_pidController = m_motor.getClosedLoopController();

    SparkMaxConfig motorConfig = new SparkMaxConfig();
    configureAsNotFollowing(motorConfig);

    // Set the conversion factor: 1 rotation * 360 = 360 degrees. Velocity
    // conversion is (similarly) 360/60 for Degrees per Second.
    //
    // Note that we are assuming that the absolute encoder is directly measuring the
    // hood angle, so the conversion factor is just 360 degrees per rotation. If
    // there were gearing between the motor and the hood, we would need to adjust
    // the conversion factor accordingly (e.g., if there were a 2:1 gear reduction,
    // the conversion factor would be 180 degrees per rotation).
    motorConfig.absoluteEncoder
        .positionConversionFactor(360.0)
        .velocityConversionFactor(360.0 / 60.0);

    // Configure the PID controller for position control, using the absolute encoder
    // as the feedback sensor. We also set an output range to prevent the hood from
    // moving too fast, which could cause mechanical stress or overshooting.
    final double minOutput = -0.5;
    final double maxOutput = 0.5;
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(kP).i(kI).d(kD)
        .outputRange(minOutput, maxOutput);

    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPosition(Angle targetAngle) {
    final double positionDegrees = targetAngle.in(Degrees);
    final double clampedPosition = MathUtil.clamp(positionDegrees, getMinAngle().in(Degrees),
        getMaxAngle().in(Degrees));
    m_pidController.setSetpoint(clampedPosition, SparkMax.ControlType.kPosition);
  }

  @Override
  public Angle getCurrentPosition() {
    return Degrees.of(m_absoluteEncoder.getPosition());
  }

  @Override
  public Angle getTargetAngle() {
    return Degrees.of(m_pidController.getSetpoint());
  }

  @Override
  public Angle getMinAngle() {
    return m_config.minAngle();
  }

  @Override
  public Angle getMaxAngle() {
    return m_config.maxAngle();
  }

  //
  // Closeable implementation (required by ISubsystem)
  //

  @Override
  public void close() throws IOException {
    m_motor.close();
  }
}
