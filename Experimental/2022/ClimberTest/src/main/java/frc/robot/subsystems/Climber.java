// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Simple design for control of the 2022 climber hardware, which is expected to
 * be built using "ThriftyBot extending climber kits".
 * 
 * Under this approach, a pair of retractable "arms" are used", which are pulled
 * down against a set of springs (when not climbing) and/or the weight of the
 * robot (when hanging) by a cable that is driven by a winch (controlled by this
 * subsystem).
 * 
 * I'm also assuming that there will be some sort of an encoder (or some other
 * way of identifying when the arms are fully let out/retracted), so that the
 * commands using the subsystem can know when to stop spinning the winch.
 * 
 * @see https://www.thethriftybot.com/bearings/Round-2-Pre-Order-Thrifty-Telescoping-Tube-Kit-p416413760
 */
public class Climber extends SubsystemBase {

  private static final double NORMAL_MOTOR_SPEED_PERCENT = 0.60;
  private static final double ADJUSTMENT_MOTOR_SPEED_PERCENT = NORMAL_MOTOR_SPEED_PERCENT * 0.5;

  private static final boolean LIMIT_SWITCHES_INSTALLED = false;

  /** Helper to allow locking motors, without keeping them as class members. */
  private interface Locker {
    void setLocked(boolean tf);
  }

  /** Helper to allow fetching encoder count from a motor. */
  private interface MotorPositionFetcher {
    double getPosition();
  }

  /** Possible motions of the climber arms. */
  public enum Motion {
    None, Extending, Retracting
  }

  public enum Side {
    None, Both, Left, Right
  }

  private final DigitalInput m_upperLimitSwitch = LIMIT_SWITCHES_INSTALLED
      ? new DigitalInput(Constants.DIO.CLIMBER_UPPER_LIMIT_SWITCH_ID)
      : null;
  private final DigitalInput m_lowerLimitSwitch = LIMIT_SWITCHES_INSTALLED
      ? new DigitalInput(Constants.DIO.CLIMBER_LOWER_LIMIT_SWITCH_ID)
      : null;

  /** Current motion of the climber arms. */
  private Motion m_currentMode = Motion.None;

  private Side m_currentSide = Side.None;

  private final MotorController m_leftMotor;
  private final MotorController m_rightMotor;
  private final Locker m_motorLocker;
  private final MotorPositionFetcher m_leftPositionFetcher;
  private final MotorPositionFetcher m_rightPositionFetcher;

  /**
   * Creates a new Climber.
   * 
   * Note: motors are configured in 'locked' mode by default.
   */
  public Climber() {

    /////////////////////////////////////////////////////////////////
    // Set up motors.
    var leftMotor = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_CLIMBER_MOTOR_ID,
        MotorType.kBrushless);
    var rightMotor = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_CLIMBER_MOTOR_ID,
        MotorType.kBrushless);

    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;

    /////////////////////////////////////////////////////////////////
    // Set up locking support.
    m_motorLocker = (boolean tf) -> {
      IdleMode mode = tf ? IdleMode.kBrake : IdleMode.kCoast;
      leftMotor.setIdleMode(mode);
      rightMotor.setIdleMode(mode);
    };
    m_motorLocker.setLocked(true);

    /////////////////////////////////////////////////////////////////
    // Set up support for reporting positions (for debugging).
    var leftEncoder = leftMotor.getEncoder();
    m_leftPositionFetcher = () -> {
      return leftEncoder.getPosition();
    };

    var rightEncoder = rightMotor.getEncoder();
    m_rightPositionFetcher = () -> {
      return rightEncoder.getPosition();
    };
  }

  public void enableBrakingMode(boolean tf) {
    m_motorLocker.setLocked(tf);
  }

  public boolean isFullyRetracted() {
    if (m_lowerLimitSwitch != null) {
      // According to docs, limit switches report true iff the switch is *open*.
      return !m_lowerLimitSwitch.get();
    }

    return false;
  }

  public boolean isFullyExtended() {
    if (m_upperLimitSwitch != null) {
      // According to docs, limit switches report true iff the switch is *open*.
      return !m_upperLimitSwitch.get();
    }

    return false;
  }

  public void holdPosition() {
    enableBrakingMode(true);
    stop();

    /*
     * [Notes from discussion among coaches on 28Jan:]
     * 
     * I think that something we'll also need to look at is how we're going to hold
     * ourselves aloft. Will we need to keep the motor running, or will we work with
     * a motor that can be put into a "brake" mode (like the SIMS)?
     * 
     * In the former case, we'd have to worry about burning the motor out unless
     * it's sufficiently "beefy". But if it *is* that beefy, then we might need to
     * worry about power draw (holding up against a 100+ pound robot, or wherever we
     * come in at) and/or whether we're going to have to play some games to keep it
     * from retracting too far and punching through something at the bottom (though
     * this would hopefully be something we can design against).
     * 
     * In the latter case, we'd need to make sure that the "brake" mode could hold
     * up against the pull of our weight.
     * 
     * Either option will have some implications for s/w.
     * 
     * It's worth noting that Nike was able to stay up without the ratchet, with a
     * 50:1 gearbox. But she was lighter than what Sally is expected to weigh by at
     * least a bit.
     */
  }

  /** Begins extending the arms, iff they aren't already fully extended. */
  public void extendArms() {
    if (isFullyExtended()) {
      // What if we were just now retracting?
      stop();
      return;
    }

    m_currentMode = Motion.Extending;
    m_currentSide = Side.Both;
    m_leftMotor.set(NORMAL_MOTOR_SPEED_PERCENT);
    m_rightMotor.set(NORMAL_MOTOR_SPEED_PERCENT);
  }

  /** Begins retracting the arms, iff they aren't already fully retracted. */
  public void retractArms() {
    if (isFullyRetracted()) {
      // What if we were just now extending?
      stop();
      return;
    }

    m_currentMode = Motion.Retracting;
    m_leftMotor.set(-NORMAL_MOTOR_SPEED_PERCENT);
    m_rightMotor.set(-NORMAL_MOTOR_SPEED_PERCENT);
    m_currentSide = Side.Both;
  }

  /**
   * Support function, used to help "rebalance" the two climber arms.
   * 
   * @param left
   */
  public void retractSingleArm(boolean left) {
    m_currentMode = Motion.Retracting;
    if (left) {
      m_leftMotor.set(-ADJUSTMENT_MOTOR_SPEED_PERCENT);
      m_currentSide = Side.Left;
    } else {
      m_rightMotor.set(-ADJUSTMENT_MOTOR_SPEED_PERCENT);
      m_currentSide = Side.Right;
    }
  }

  /**
   * Support function, used to help "rebalance" the two climber arms.
   * 
   * @param left
   */
  public void extendSingleArm(boolean left) {
    m_currentMode = Motion.Extending;
    if (left) {
      m_leftMotor.set(ADJUSTMENT_MOTOR_SPEED_PERCENT);
      m_currentSide = Side.Left;
    } else {
      m_rightMotor.set(ADJUSTMENT_MOTOR_SPEED_PERCENT);
      m_currentSide = Side.Right;
    }
  }

  /**
   * Stops the arms moving.
   * 
   * Note that this does *not* alter braking behavior: if the motors are set to
   * "coast", then the arms will not be stopped from moving in response to
   * external forces.
   * 
   * @see #holdPosition()
   */
  public void stop() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
    m_currentMode = Motion.None;
    m_currentSide = Side.None;
  }

  /**
   * @return true iff the motors are stopped (either via client request, or
   *         because we've hit some limit of concern)
   */
  public boolean isStopped() {
    return (m_currentMode == Motion.None);
  }

  /**
   * Checks to see if we've reached a limit on the current direction in which the
   * climber arms are moving. (If so, we will stop them, and update the current
   * mode to reflect this.)
   * 
   * Note: this method will be called once per scheduler run.
   */
  @Override
  public void periodic() {
    super.periodic();

    if (LIMIT_SWITCHES_INSTALLED) {
      switch (m_currentMode) {
        case Extending:
          if (isFullyExtended()) {
            System.err.println(">>> Climber is fully extended: stopping operation.");
            stop();
          }
          break;

        case Retracting:
          if (isFullyRetracted()) {
            System.err.println(">>> Climber is fully retracted: stopping operation.");
            stop();
          }
          break;

        case None:
          // Nothing to do....
          break;
      }
    }

    // Status reporting (to use in debugging tests/hardware).
    SmartDashboard.putString("Mode", m_currentMode.toString());
    SmartDashboard.putString("Active side", m_currentSide.toString());
    if (LIMIT_SWITCHES_INSTALLED) {
      SmartDashboard.putString("Upper limit", m_upperLimitSwitch.get() ? "open" : "closed");
      SmartDashboard.putString("Lower limit", m_lowerLimitSwitch.get() ? "open" : "closed");
    }
    SmartDashboard.putString("Left pos", Double.toString(m_leftPositionFetcher.getPosition()));
    SmartDashboard.putString("Right pos", Double.toString(m_rightPositionFetcher.getPosition()));
  }
}
