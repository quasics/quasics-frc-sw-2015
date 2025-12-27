// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.*;
import static frc.robot.utils.RevSupportFunctions.configureForRadians;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.utils.RobotConfigs.ArmFeedForwardConfig;
import frc.robot.utils.RobotConfigs.PIDConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Subsystem representing the arm on our real 2025 robot.
 *
 * TODO: Test this class on the real robot.
 */
public class Arm extends SubsystemBase implements ISingleJointArm {
  /** Motor controller running the arm. */
  protected final SparkMax m_motorController =
      new SparkMax(Constants.OtherCanIds.ARM_LEADER_ID, MotorType.kBrushless);
  ;

  /** Encoder providing readings from the through-bore. */
  protected final AbsoluteEncoder m_throughBoreEncoder = m_motorController.getAbsoluteEncoder();

  /**
   * Reference/target position for arm. (If null, no target position has been
   * set.)
   */
  protected Angle m_referencePosition = null;

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
    setName(SUBSYSTEM_NAME);

    final PIDConfig pidConfig = config.arm().pid();
    m_armPIDController = new PIDController(pidConfig.kP(), pidConfig.kI(), pidConfig.kD());
    m_armPIDController.setTolerance(0.5, 1);

    final ArmFeedForwardConfig ffConfig = config.arm().feedForward();
    m_feedForward = new ArmFeedforward(
        ffConfig.kS().in(Volts), ffConfig.kG().in(Volts), ffConfig.kV(), ffConfig.kA());

    // Through-bore encoder configuration settings.
    final AbsoluteEncoderConfig throughBoreConfig = new AbsoluteEncoderConfig();
    throughBoreConfig.inverted(true);
    configureForRadians(throughBoreConfig);

    // SparkMax configuration settings.
    final SparkMaxConfig controllerConfig = new SparkMaxConfig();
    controllerConfig.inverted(false).apply(throughBoreConfig);

    // Apply the configuration settings.
    m_motorController.configure(
        controllerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Gets the current angle of the arm.
   *
   * @return current position in radians
   */
  public Angle getPivotAngle() {
    return Radians.of(m_throughBoreEncoder.getPosition());
  }

  /**
   * Gets the current (rotational) velocity of the arm.
   *
   * @return current velocity in radians/sec
   */
  public AngularVelocity getPivotVelocity() {
    return RadiansPerSecond.of(m_throughBoreEncoder.getVelocity());
  }

  /**
   * Drives the arm to the setpoint (via PID/Feedforward), if a reference position
   * has been set.
   */
  protected void driveArmToSetpoint() {
    final boolean NOISY = false;

    if (m_referencePosition == null) {
      return;
    }

    final double currentAngleRadians = getPivotAngle().in(Radians);
    final double currentVelocity_radiansPerSec = getPivotVelocity().in(RadiansPerSecond);
    double pidOutput =
        m_armPIDController.calculate(currentAngleRadians, m_referencePosition.in(Radians));
    double feedForwardOutput =
        m_feedForward.calculate(currentAngleRadians, currentVelocity_radiansPerSec);
    double output = feedForwardOutput + pidOutput;
    m_motorController.setVoltage(output);

    if (NOISY) {
      System.out.printf("pid: %.02f, feedforward: %.02f, output: %.02f, setpoint: %.02f\n",
          pidOutput, feedForwardOutput, output, m_referencePosition.in(Degrees));
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

  /**
   * Angle returned by the encoder when the arm is fully extended (horizontal).
   */
  public static final Angle ARM_DOWN_ANGLE = Degrees.of(0);

  /**
   * Angle returned by the encoder when the arm is fully upright.
   */
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
