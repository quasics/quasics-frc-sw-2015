// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Provides pretty basic control for the climber hardware, including a "safety
 * mode" that is intended to prevent them from being wound too far.
 * 
 * TODO: Test this code.
 */
public class Climber extends SubsystemBase {
  public static final double MAX_SAFE_REVOLUTIONS = 6;
  static final int TICKS_PER_REVOLUTION = 42;
  static final int LEFT_CLIMBER_CAN_ID = 5;
  static final int RIGHT_CLIMBER_CAN_ID = 6;
  static final double EXTENSION_SPEED = -1.0;
  static final double RETRACTION_SPEED = 1.0;

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
  public Climber() {
    setName(getClass().getSimpleName());
    m_leftEncoder.setPositionConversionFactor(1.0 / TICKS_PER_REVOLUTION);
    m_rightEncoder.setPositionConversionFactor(1.0 / TICKS_PER_REVOLUTION);

    m_leftClimber.setIdleMode(IdleMode.kBrake);
    m_rightClimber.setIdleMode(IdleMode.kBrake);
  }

  public void enableSafeMode(boolean tf) {
    m_safetyOn = tf;
  }

  public boolean isSafeModeEnabled() {
    return m_safetyOn;
  }

  public void resetLeftEncoder() {
    m_leftEncoder.setPosition(0);
  }

  public void resetRightEncoder() {
    m_rightEncoder.setPosition(0);
  }

  public double getLeftRevolutions() {
    return m_leftEncoder.getPosition();
  }

  public double getRightRevolutions() {
    return m_rightEncoder.getPosition();
  }

  public Mode getLeftMode() {
    return m_leftMode;
  }

  public Mode getRightMode() {
    return m_rightMode;
  }

  public void stop() {
    stopLeftClimber();
    stopRightClimber();
  }

  public void stopLeftClimber() {
    m_leftClimber.set(0);
    m_leftMode = Mode.Stopped;
  }

  public void stopRightClimber() {
    m_rightClimber.set(0);
    m_rightMode = Mode.Stopped;
  }

  public boolean extendLeftClimber() {
    if (m_safetyOn && getLeftRevolutions() >= MAX_SAFE_REVOLUTIONS) {
      stopLeftClimber();
      return false;
    }

    m_leftClimber.set(EXTENSION_SPEED);
    m_leftMode = Mode.Extending;
    return true;
  }

  public boolean retractLeftClimber() {
    if (m_safetyOn && getLeftRevolutions() <= 0) {
      stopLeftClimber();
      return false;
    }

    m_leftClimber.set(RETRACTION_SPEED);
    m_leftMode = Mode.Retracting;
    return true;
  }

  public boolean extendRightClimber() {
    if (m_safetyOn && getRightRevolutions() >= MAX_SAFE_REVOLUTIONS) {
      stopRightClimber();
      return false;
    }

    m_rightClimber.set(EXTENSION_SPEED);
    m_rightMode = Mode.Extending;
    return true;
  }

  public boolean retractRightClimber() {
    if (m_safetyOn && getRightRevolutions() <= 0) {
      stopRightClimber();
      return false;
    }

    m_rightClimber.set(RETRACTION_SPEED);
    m_rightMode = Mode.Retracting;
    return true;
  }

  @Override
  public void periodic() {
    if (m_safetyOn) {
      if ((m_leftMode == Mode.Extending && getLeftRevolutions() >= MAX_SAFE_REVOLUTIONS)
          || (m_leftMode == Mode.Retracting && getLeftRevolutions() <= 0)) {
        stopLeftClimber();
      }
      if ((m_rightMode == Mode.Extending && getRightRevolutions() >= MAX_SAFE_REVOLUTIONS)
          || (m_rightMode == Mode.Retracting && getRightRevolutions() <= 0)) {
        stopRightClimber();
      }
    }
  }
}
