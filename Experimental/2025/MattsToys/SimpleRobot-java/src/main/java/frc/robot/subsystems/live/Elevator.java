// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.DioIds;
import frc.robot.Constants.OtherCanIds;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.abstracts.AbstractElevator;
import frc.robot.utils.RobotConfigs.ElevatorConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Subsystem representing the elevator on our real 2025 robot.
 * 
 * Note: per Ethan, the elevator is currently configures such that "up" for the
 * leader motor is negative. This feels... wrong, and is something that can be
 * fixed (e.g., by switching to using the current "follower" as the leader, or
 * reversing "inverted" values), but until I've had a chance to actually try
 * out/debug the code on the real hardware, I'm not going to mess with it.
 */
public class Elevator extends AbstractElevator {
  private final SparkMax m_leader = new SparkMax(OtherCanIds.LEADER_ELEVATOR_ID, MotorType.kBrushless);
  private final DigitalInput m_limitSwitchUp = new DigitalInput(DioIds.ELEVATOR_LIMIT_SWITCH_UP);
  private final DigitalInput m_limitSwitchDown = new DigitalInput(DioIds.ELEVATOR_LIMIT_SWITCH_DOWN);

  private RelativeEncoder m_encoder = m_leader.getEncoder();
  private TrivialEncoder m_wrappedEncoder = new SparkMaxEncoderWrapper(m_encoder);

  private final PIDController m_pid;
  private final ElevatorFeedforward m_feedforward;

  /**
   * Constructor.
   * 
   * @param config robot configuration
   */
  public Elevator(RobotConfig config) {
    // Configure the PID/FF controllers.
    ElevatorConfig elevatorConfig = config.elevator();
    m_pid = new PIDController(
        elevatorConfig.pid().kP(),
        elevatorConfig.pid().kI(),
        elevatorConfig.pid().kD());
    m_pid.setTolerance(0.02); // within 2cm is fine
    m_feedforward = new ElevatorFeedforward(
        elevatorConfig.feedForward().kS().in(Volts),
        elevatorConfig.feedForward().kG().in(Volts),
        elevatorConfig.feedForward().kV(),
        elevatorConfig.feedForward().kA());

    // Configure the "follower" motor.
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(m_leader, true);
    try (SparkMax follower = new SparkMax(OtherCanIds.FOLLOWER_ELEVATOR_ID, MotorType.kBrushless)) {
      follower.configure(followerConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    }

    // Configure the primary (leader) motor.
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.inverted(false);

    configureForDistance(leaderConfig);

    m_leader.configure(
        leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  protected void configureForRpm(SparkMaxConfig sparkMaxConfig) {
    sparkMaxConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
  }

  // TODO: Update these constants to match the real hardware.
  //
  // Per Sean, "it’s a motor / gearbox that then runs a chain/sprocket. The
  // sprockets are 1:1 on the top:bottom, with the gearing ratio being (IIRC)
  // 48:1."
  // Per Jasdeep, the pitch diameter (i.e., ) is 1.75".
  //
  // (For a discussion of "pitch diameter", see
  // https://docs.wcproducts.com/frc-build-system/belts-chain-and-gears/sprockets-and-chain.)
  private static final Distance kSprocketPitchDiameter = Inches.of(1.75);
  private static final double kGearingRatio = 48;
  private static final double kEncoderMetersPerRotation = kSprocketPitchDiameter.abs(Meters) / kGearingRatio;

  protected void configureForDistance(SparkMaxConfig sparkMaxConfig) {
    sparkMaxConfig.encoder
        .positionConversionFactor(kEncoderMetersPerRotation)
        .velocityConversionFactor(kEncoderMetersPerRotation / 60);
  }

  @Override
  public void setTargetPosition(TargetPosition targetPosition) {
    if (targetPosition != m_target) {
      m_pid.reset();
    }

    super.setTargetPosition(targetPosition);
  }

  @Override
  public void periodic() {
    super.periodic();

    // Safety condition: if we're at the top or bottom, stop.
    //
    // Note that we don't do this when we're under PID control (for now, at
    // least).
    if ((m_mode == Mode.Extending && isAtTop()) ||
        (m_mode == Mode.Retracting && isAtBottom())) {
      stop();
    }

    System.out.println(NAME + " - " +
        "mode" + m_mode +
        " height: " + getHeight_impl() +
        " atTop: " + isAtTop() +
        " atBottom: " + isAtBottom());
  }

  @Override
  public boolean atTargetPosition() {
    if (m_target == TargetPosition.DontCare) {
      return true;
    } else {
      return m_pid.atSetpoint();
    }
  }

  @Override
  protected void updateMotor_impl() {
    var voltage = calculateMotorVoltage(getPositionForTarget(m_target), m_wrappedEncoder, m_pid, m_feedforward);
    m_leader.setVoltage(voltage);
  }

  @Override
  protected void resetEncoder_impl() {
    m_encoder.setPosition(0);
  }

  @Override
  protected Distance getHeight_impl() {
    return m_wrappedEncoder.getPosition();
  }

  @Override
  protected void stop_impl() {
    m_leader.set(0);
    m_target = TargetPosition.DontCare;
  }

  private static final double ELEVATOR_SPEED_UP_PERCENT = 0.5;
  private static final double ELEVATOR_SPEED_DOWN_PERCENT = -0.5;

  @Override
  protected void extend_impl() {
    moveSafely(ELEVATOR_SPEED_UP_PERCENT);
    m_target = TargetPosition.DontCare;
  }

  @Override
  protected void retract_impl() {
    moveSafely(ELEVATOR_SPEED_DOWN_PERCENT);
    m_target = TargetPosition.DontCare;
  }

  /**
   * Value returned from the limits switches when they are activated by the
   * elevator reaching them. (Per Ethan, they are currently "defaulting closed",
   * and will go open when they are triggered.)
   */
  final static boolean SWITCH_ACTIVATED_VALUE = false;

  /**
   * @return true if the elevator is at the top of its path (based on limit
   *         switch)
   */
  public boolean isAtTop() {
    return m_limitSwitchUp.get() == SWITCH_ACTIVATED_VALUE;
  }

  /**
   * @return true if the elevator is at the bottom of its path (based on limit
   *         switch)
   */
  public boolean isAtBottom() {
    return m_limitSwitchDown.get() == SWITCH_ACTIVATED_VALUE;
  }

  /**
   * @return if the elevator is safe to move, given the current speed (with sign
   *         indicating direction)
   */
  private boolean ableToMove(double speed) {
    if (isAtTop()) {
      return (speed >= 0); // We can move *down* (positive values), but not up.
    }
    if (isAtBottom()) {
      return (speed <= 0); // We can move *up* (negative values), but not down.
    }

    // If we're at neither extreme, we can move in either direction.
    return true;
  }

  /**
   * Sets the elevator speed/direction, if it safe to do so; if not, stops the
   * elevator.
   */
  private void moveSafely(double speed) {
    if (ableToMove(speed)) {
      m_leader.set(speed);
    } else {
      stop();
    }
  }

  protected Distance getPositionForTarget(TargetPosition position) {
    switch (position) {
      case DontCare:
        return m_wrappedEncoder.getPosition();
      // TODO: Check these values.
      case Bottom:
        return Meters.of(0);
      case L1:
        return Meters.of(-74 * kEncoderMetersPerRotation);
      case L2:
        return Meters.of(-145 * kEncoderMetersPerRotation);
      case Top:
        return Meters.of(-194 * kEncoderMetersPerRotation);
    }

    System.err.println("Unrecognized target position: " + position);
    return Meters.of(0);
  }
}
