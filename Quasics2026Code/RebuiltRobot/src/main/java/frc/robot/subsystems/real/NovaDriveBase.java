// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.EncoderType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.ThriftyNovaIds;
import frc.robot.hardware.actuators.ThriftyNovaMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.Pigeon2Wrapper;
import frc.robot.hardware.sensors.ThriftyEncoderWrapper;
import frc.robot.hardware.sensors.TrivialEncoder;

public class NovaDriveBase extends AbstractDrivebase {
  /** Track width (distance between left and right wheels) in meters. */
  public static final Distance TRACK_WIDTH =
      Meters.of(0.546); /* 21.5 inches (updated to 2026) */

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

  /**
   * Creates a new NovaDriveBase, using the default motor controller IDs
   * specified in Constants.CanBusIds. (This is the constructor that should be
   * used in most... if not all... cases, since it will ensure that the correct
   * motor controller IDs are used, and that the followers are correctly
   * configured to follow the leaders, etc. If you need to use different motor
   * controller IDs for some reason, then you can use the other constructor, but
   * be sure to use it correctly.)
   *
   * @see #NovaDriveBase(ThriftyNova, ThriftyNova)
   */
  public NovaDriveBase() {
    this(new ThriftyNova(ThriftyNovaIds.LEFT_LEADER_ID),
        new ThriftyNova(ThriftyNovaIds.RIGHT_LEADER_ID));
  }

  /**
   * Creates a new NovaDriveBase.
   *
   * Note that the motor controllers passed in here should be the "leader" motor
   * controllers (i.e., the ones that the followers will be configured to
   * follow). If you pass in the "follower" motor controllers here, then the
   * followers won't be correctly configured to follow the leaders, and the
   * drivebase won't work correctly. (And if you pass in a mix of leaders and
   * followers, then the behavior will be... well, it will be weird, and
   * probably not what you want.)
   *
   * @param leftController  the ThriftyNova motor controller for the left side
   * @param rightController the ThriftyNova motor controller for the right side
   */
  public NovaDriveBase(
      ThriftyNova leftController, ThriftyNova rightController) {
    super(new ThriftyNovaMotorControllerPlus(leftController),
        new ThriftyNovaMotorControllerPlus(rightController), TRACK_WIDTH);

    // Configure followers to follow the leaders.
    configureMotorControllersForFollowing(
        ThriftyNovaIds.LEFT_LEADER_ID, ThriftyNovaIds.LEFT_FOLLOWER_ID);
    configureMotorControllersForFollowing(
        ThriftyNovaIds.RIGHT_LEADER_ID, ThriftyNovaIds.RIGHT_FOLLOWER_ID);

    //
    // Configure the leading motors.
    //

    // Configure the leaders so that they are *not* a follower of anything.
    //
    // This is important to do to ensure that the leader motor controllers are
    // correctly configured even if they get swapped out.
    leftController.follow(0);
    rightController.follow(0);

    // Configure the encoder type. (Note that only the leaders need to know
    // this, since we won't read encoder data from the followers.)
    ThriftyNovaConfig configLeft = new ThriftyNovaConfig();
    configLeft.encoderType = EncoderType.INTERNAL;
    configLeft.inverted = false;

    ThriftyNovaConfig configRight = new ThriftyNovaConfig();
    configRight.encoderType = EncoderType.INTERNAL;
    configRight.inverted = true;

    // Apply the configuration settings to the motors.
    leftController.applyConfig(configLeft);
    rightController.applyConfig(configRight);

    //
    // Set up the TrivialEncoders we'll use to handle accessing the data from
    // the motors.
    //
    final Distance wheelDiam = Constants.WHEEL_RADIUS.times(2);
    m_leftEncoder = new ThriftyEncoderWrapper(
        leftController, wheelDiam, Constants.DRIVEBASE_GEAR_RATIO);
    m_rightEncoder = new ThriftyEncoderWrapper(
        leftController, wheelDiam, Constants.DRIVEBASE_GEAR_RATIO);

    //
    // Set up the gyro.
    //
    Pigeon2 pigeon = new Pigeon2(Constants.CanBusIds.PIGEON2_CAN_ID);
    m_gryo = new Pigeon2Wrapper(pigeon);
  }

  /**
   * Configures a follower ThriftyNova motor controller to follow a leader
   * ThriftyNova
   * motor controller.
   *
   * Note that this is important to do in code (instead of just setting the
   * followers to follow the leaders simply be using a configration app) to
   * ensure that the followers will be correctly configured even if a motor
   * controller gets swapped out (e.g., if a controller gets damaged and needs
   * to be replaced, or if we need to swap a controller from one side of the
   * drivebase to the other for some reason, etc.).
   *
   * @param leaderId   CAN ID of the leader ThriftyNova motor controller
   * @param followerId CAN ID of the ThriftyNova motor controller that should be
   *                   configured to follow the leader
   */
  private void configureMotorControllersForFollowing(
      int leaderId, int followerId) {
    try (ThriftyNova follower = new ThriftyNova(followerId)) {
      // Configure the motor to follow the leader
      //
      // Pass second parameter of 'true' to invert the direction (i.e., to run
      // in the opposite direction as the leader): this isn't wanted for the
      // drive base.
      follower.follow(leaderId);
    } catch (Exception e) {
      // Something went wrong when releasing the follower: log the error.
      e.printStackTrace();
    }
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
