// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

  private enum Motion {
    kNone, kExtending, kRetracting
  }

  // TODO: Make this a non-arbitrary value. :-)
  public static final double MAX_EXTENSION_INCHES = 100.0;

  private static final double EXTENDING_SPEED_PERCENT = 0.25;
  private static final double RETRACTING_SPEED_PERCENT = 0.25;

  private final CANSparkMax m_leftMotor = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_CLIMBER_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_CLIMBER_MOTOR_ID,
      MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final MotorControllerGroup m_motors;

  private Motion m_currentMode = Motion.kNone;

  /** Creates a new Climber. */
  public Climber() {
    // TODO: Figure out which of the motors actually *is* inverted (if any).
    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    // TODO: Add code to convert encoder positions from revolutions to inches.
    // (Though this may not be possible, since the cable will wrap around itself
    // somewhat unpredicably: we may need to consider averaging, or else using some
    // other type of sensor, such as Hall effect, to detect arm positions.)

    m_motors = new MotorControllerGroup(m_leftMotor, m_rightMotor);

    resetPosition();
  }

  public void enableBrakingMode(boolean tf) {
    IdleMode mode = tf ? IdleMode.kBrake : IdleMode.kCoast;
    m_rightMotor.setIdleMode(mode);
    m_rightMotor.setIdleMode(mode);
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

  private void resetPosition() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getPosition() {
    return m_leftEncoder.getPosition();
  }

  public void extendArms() {
    if (armsInBounds()) {
      m_motors.set(EXTENDING_SPEED_PERCENT);
      m_currentMode = Motion.kExtending;
    }
  }

  /**
   * 
   */
  public void retractArms() {
    if (armsInBounds()) {
      m_motors.set(RETRACTING_SPEED_PERCENT);
      m_currentMode = Motion.kRetracting;
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
    m_motors.stopMotor();
    m_currentMode = Motion.kNone;
  }

  /**
   * @return true iff the motors are stopped (either via client request, or
   *         because we've hit a soft limit)
   */
  public boolean isStopped() {
    return (m_currentMode == Motion.kNone);
  }

  private boolean armsInBounds() {
    // TODO: Add code to support "soft stops" on the motors.
    // These decisions will be based on m_currentMode and either data from position,
    // or via alternate signals (e.g., limit switches, Hall effect sensors, etc.).
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!armsInBounds()) {
      stop();
    }
  }
}
