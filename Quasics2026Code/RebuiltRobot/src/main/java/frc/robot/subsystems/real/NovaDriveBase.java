// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.Constants.CanBusIds.ThriftyNovaIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.ThriftyEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.sensors.ThriftyEncoderWrapper;

public class NovaDriveBase extends AbstractDrivebase {
  // TODO: add thriftynova support. (This might be done in a derived class, or
  // be based on some information about the robot's configuration. I'd recommend
  // the former approach.)

  // TODO: Change these to use the encoders that are associated with the real
  // hardware (i.e., either the relative encoders that are built into the Spark
  // Max hardware, or else the functions that are built into the ThriftyNova
  // motor controller class).
  //
  // Note that Mr. Healy has updated the "TrivialEncoder" class (and some
  // derived classes) so that it can be used with Thrifty Novas (new code this
  // year), as well as the Spark Max controllers, etc. (This stuff is in the
  // sample code under "Experimental/2026/MattsToys/SimulationSupport".)
  private final TrivialEncoder m_leftEncoder;
  private final TrivialEncoder m_rightEncoder;

  private final IGyro m_gryo;

  @Override
  protected final IGyro getGyro() {
    return m_gryo;
  }

  @Override
  protected final TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected final TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /** Creates a new RealDrivebase. */
  public NovaDriveBase(ThriftyNova leftMotorController, ThriftyNova rightMotorController) {
    super(leftMotorController, rightMotorController);

    // Configure followers to follow the leaders.
    final ThriftyNova leftfollower = new ThriftyNova(ThriftyNovaIds.LEFT_FOLLOWER_ID);
    final ThriftyNova rightfollower = new ThriftyNova(ThriftyNovaIds.RIGHT_FOLLOWER_ID);

    configureMotorControllersForFollowing(ThriftyNovaIds.LEFT_LEADER_ID, leftfollower);
    configureMotorControllersForFollowing(ThriftyNovaIds.RIGHT_LEADER_ID, rightfollower);

    // Configure the encoders.
    ThriftyNovaConfig config = new ThriftyNovaConfig();
    config.encoderType = EncoderType.INTERNAL;
    leftfollower.applyConfig(config);
    rightfollower.applyConfig(config);

    final Distance wheelDiam = Meters.of(2.0 * Constants.wheelRadius.in(Meters));
    m_leftEncoder = new ThriftyEncoderWrapper(leftMotorController, wheelDiam);
    m_rightEncoder = new ThriftyEncoderWrapper(leftMotorController, wheelDiam);

    // Configure the gyro.
    //
    // TODO: Switch this to use the gyro that we're actually going to be using on
    // the real robot.
    //
    // FINDME(Robert): This needs to be updated, since we're not actually going to
    // be using an AnalogGyro for the real robot. (We'll probably be using a
    // Pigeon2.)
    AnalogGyro gyro = new AnalogGyro(0);
    m_gryo = IGyro.wrapGyro(gyro);
  }

  public NovaDriveBase() {
    this(new ThriftyNova(ThriftyNovaIds.LEFT_LEADER_ID), new ThriftyNova(ThriftyNovaIds.RIGHT_LEADER_ID));
  }

  /**
   * Configures a follower SparkMax motor controller to follow a leader SparkMax
   * motor controller.
   * 
   * Note that this is important to do in code (instead of just setting the
   * followers to follow the leaders simply be using a configration app) to ensure
   * that the followers will be correctly configured even if a motor controller
   * gets swapped out (e.g., if a controller gets damaged and needs to be
   * replaced, or if we need to swap a controller from one side of the drivebase
   * to the other for some reason, etc.).
   * 
   * @param leader   leader SparkMax motor controller that the follower should
   *                 follow
   * @param follower SparkMax motor controller that should be configured to follow
   *                 the leader
   */
  private void configureMotorControllersForFollowing(int leader, ThriftyNova follower) {

    // Configure the motor to follow the leader
    // Pass second parameter of 'true' to invert the direction
    follower.follow(leader);

  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
