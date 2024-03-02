// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * Provides pretty basic control for the climber hardware, including a "safety
 * mode" that is intended to prevent them from being wound too far.
 * 
 * TODO: Test this code.
 */
public class RealClimber extends AbstractClimber {
  static final int TICKS_PER_REVOLUTION = 42;
  static final int LEFT_CLIMBER_CAN_ID = 5;
  static final int RIGHT_CLIMBER_CAN_ID = 6;

  public enum Mode {
    Stopped, Extending, Retracting
  }

  final CANSparkMax m_leftClimber = new CANSparkMax(LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightClimber = new CANSparkMax(LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);

  final RelativeEncoder m_leftEncoder = m_leftClimber.getEncoder();
  final RelativeEncoder m_rightEncoder = m_rightClimber.getEncoder();

  Mode m_leftMode = Mode.Stopped;
  Mode m_rightMode = Mode.Stopped;

  boolean m_safetyOn = true;

  /** Creates a new Climber. */
  public RealClimber() {
    setName(getClass().getSimpleName());
    m_leftEncoder.setPositionConversionFactor(1.0 / TICKS_PER_REVOLUTION);
    m_rightEncoder.setPositionConversionFactor(1.0 / TICKS_PER_REVOLUTION);

    m_leftClimber.setIdleMode(IdleMode.kBrake);
    m_rightClimber.setIdleMode(IdleMode.kBrake);
  }

  @Override
  protected void resetLeftEncoder_HAL() {
    m_leftEncoder.setPosition(0);
  }

  @Override
  protected void resetRightEncoder_HAL() {
    m_rightEncoder.setPosition(0);
  }

  @Override
  protected double getLeftRevolutions_HAL() {
    return m_leftEncoder.getPosition();
  }

  @Override
  protected double getRightRevolutions_HAL() {
    return m_rightEncoder.getPosition();
  }

  @Override
  protected void stopLeftClimber_HAL() {
    m_leftClimber.set(0);
  }

  @Override
  protected void stopRightClimber_HAL() {
    m_rightClimber.set(0);
  }

  @Override
  protected void extendLeftClimber_HAL() {
    m_leftClimber.set(EXTENSION_SPEED);
  }

  @Override
  protected void retractLeftClimber_HAL() {
    m_leftClimber.set(RETRACTION_SPEED);
  }

  @Override
  protected void extendRightClimber_HAL() {
    m_rightClimber.set(EXTENSION_SPEED);
  }

  @Override
  protected void retractRightClimber_HAL() {
    m_rightClimber.set(RETRACTION_SPEED);
  }
}
