// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Provides pretty basic control for the climber hardware, including a "safety
 * mode" that is intended to prevent them from being wound too far.
 * 
 * TODO: Test this code.
 */
public abstract class AbstractClimber extends SubsystemBase {
  public static final double MAX_SAFE_REVOLUTIONS = 6;

  public enum Mode {
    Stopped, Extending, Retracting
  }

  Mode m_leftMode = Mode.Stopped;
  Mode m_rightMode = Mode.Stopped;

  boolean m_safetyOn = true;

  /** Creates a new Climber. */
  public AbstractClimber() {
    setName(getClass().getSimpleName());
  }

  public void enableSafeMode(boolean tf) {
    m_safetyOn = tf;
  }

  public boolean isSafeModeEnabled() {
    return m_safetyOn;
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
    stopLeftClimber_HAL();
    m_leftMode = Mode.Stopped;
  }

  public void stopRightClimber() {
    stopRightClimber_HAL();
    m_rightMode = Mode.Stopped;
  }

  public boolean extendLeftClimber() {
    if (m_safetyOn && getLeftRevolutions_HAL() >= MAX_SAFE_REVOLUTIONS) {
      stopLeftClimber();
      return false;
    }

    extendLeftClimber_HAL();
    m_leftMode = Mode.Extending;
    return true;
  }

  public boolean retractLeftClimber() {
    if (m_safetyOn && getLeftRevolutions_HAL() <= 0) {
      stopLeftClimber();
      return false;
    }

    retractLeftClimber_HAL();
    m_leftMode = Mode.Retracting;
    return true;
  }

  public boolean extendRightClimber() {
    if (m_safetyOn && getRightRevolutions_HAL() >= MAX_SAFE_REVOLUTIONS) {
      stopRightClimber();
      return false;
    }

    extendRightClimber_HAL();
    m_rightMode = Mode.Extending;
    return true;
  }

  public boolean retractRightClimber() {
    if (m_safetyOn && getRightRevolutions_HAL() <= 0) {
      stopRightClimber();
      return false;
    }

    retractRightClimber_HAL();
    m_rightMode = Mode.Retracting;
    return true;
  }

  @Override
  public void periodic() {
    if (m_safetyOn) {
      final double leftRevolutions = getLeftRevolutions_HAL();
      final double rightRevolutions = getRightRevolutions_HAL();
      if ((m_leftMode == Mode.Extending && leftRevolutions >= MAX_SAFE_REVOLUTIONS)
          || (m_leftMode == Mode.Retracting && leftRevolutions <= 0)) {
        stopLeftClimber();
      }
      if ((m_rightMode == Mode.Extending && rightRevolutions >= MAX_SAFE_REVOLUTIONS)
          || (m_rightMode == Mode.Retracting && rightRevolutions <= 0)) {
        stopRightClimber();
      }
    }
  }

  protected abstract void resetLeftEncoder_HAL();

  protected abstract void resetRightEncoder_HAL();

  protected abstract double getLeftRevolutions_HAL();

  protected abstract double getRightRevolutions_HAL();

  protected abstract void stopLeftClimber_HAL();

  protected abstract void stopRightClimber_HAL();

  protected abstract void extendLeftClimber_HAL();

  protected abstract void extendRightClimber_HAL();

  protected abstract void retractLeftClimber_HAL();

  protected abstract void retractRightClimber_HAL();
}
