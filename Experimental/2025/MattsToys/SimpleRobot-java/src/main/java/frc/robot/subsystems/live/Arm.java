// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.abstracts.AbstractSparkMaxArm;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Subsystem representing the arm on our real 2025 robot.
 */
public class Arm extends AbstractSparkMaxArm {
  /** Encoder providing readings from the through-bore. */
  protected final AbsoluteEncoder m_throughBoreEncoder;

  /** PID controller for the arm. */
  protected final PIDController m_armPIDController;

  /** Feedforward controller for the arm. */
  protected final ArmFeedforward m_feedForward;

  /**
   * Constructor.
   * 
   * @param config robot configuration
   */
  public Arm(RobotConfig config) {
    super(Constants.OtherCanIds.ARM_LEADER_ID);

    m_throughBoreEncoder = m_motorController.getAbsoluteEncoder();

    // TODO: Move PID/Feedforward values to the RobotConfig.
    m_armPIDController = new PIDController(10.0, 0.0, 0.0);
    m_armPIDController.setTolerance(0.5, 1);

    m_feedForward = new ArmFeedforward(0.2, 0.25, 0.0);

    // Through-bore encoder configuration settings.
    final AbsoluteEncoderConfig throughBoreConfig = new AbsoluteEncoderConfig();
    throughBoreConfig
        .inverted(true)
        // Convert from encoder units (revolutions/RPMs) to radians/radians per second.
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60);

    // SparkMax configuration settings.
    final SparkMaxConfig controllerConfig = new SparkMaxConfig();
    controllerConfig
        .inverted(false)
        .apply(throughBoreConfig);

    // Apply the configuration settings.
    m_motorController.configure(
        controllerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** @return current position in radians */
  public Angle getPivotAngle() {
    return Radians.of(m_throughBoreEncoder.getPosition());
  }

  /** @return current velocity in radians/sec */
  public AngularVelocity getPivotVelocity() {
    return RadiansPerSecond.of(m_throughBoreEncoder.getVelocity());
  }

  protected void driveArmToSetpoint() {
    final boolean NOISY = false;

    if (m_referencePosition == null) {
      return;
    }

    final double currentAngleRadians = getPivotAngle().in(Radians);
    final double currentVelocity_radiansPerSec = getPivotVelocity().in(RadiansPerSecond);
    double pidOutput = m_armPIDController.calculate(currentAngleRadians, m_referencePosition.in(Radians));
    double feedForwardOutput = m_feedForward.calculate(currentAngleRadians, currentVelocity_radiansPerSec);
    double output = feedForwardOutput + pidOutput;
    m_motorController.setVoltage(output);

    if (NOISY) {
      System.out.printf(
          "pid: %.02f, feedforward: %.02f, output: %.02f, setpoint: %.02f\n",
          pidOutput,
          feedForwardOutput,
          output, m_referencePosition.in(Degrees));
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    driveArmToSetpoint();
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Methods from ISingleJointArm
  ////////////////////////////////////////////////////////////////////////////////////

  // TODO: Test these values using the real robot.
  public static final Angle ARM_DOWN_ANGLE = Degrees.of(0);
  public static final Angle ARM_UP_ANGLE = Degrees.of(90);

  @Override
  public Angle getArmOutAngle() {
    return ARM_DOWN_ANGLE;
  }

  @Override
  public Angle getArmUpAngle() {
    return ARM_UP_ANGLE;
  }

  @Override
  public void setTargetPosition(Angle targetPosition) {
    m_referencePosition = targetPosition;
    // We'll pick this change up in the periodic() method.
  }
}
