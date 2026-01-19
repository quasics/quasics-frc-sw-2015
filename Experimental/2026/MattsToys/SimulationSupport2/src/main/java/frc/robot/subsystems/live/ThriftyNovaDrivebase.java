// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import frc.robot.constants.robots.QuasicsThriftyNovaConstants.QuasicsDrivebaseCanIds;
import frc.robot.hardware.actuators.ThriftyNovaMotorControllerPlus;
import frc.robot.hardware.sensors.Pigeon2Wrapper;
import frc.robot.hardware.sensors.ThriftyEncoderWrapper;
import frc.robot.subsystems.DrivebaseBase;
import frc.robot.util.RobotConfigs.DriveConfig;

/**
 * Sample implementation of the drivebase functionality, based on CAN-based
 * Thrify Nova controllers.
 */
public class ThriftyNovaDrivebase extends DrivebaseBase {
  /**
   * Basic constructor.
   *
   * Note that this passes through to a more detailed constructor, so that we
   * can configure the leaders/followers without mucking around with unsafe
   * casts, etc.
   *
   * @param config drive base configuration
   */
  public ThriftyNovaDrivebase(DriveConfig config) {
    this(config,
        new ThriftyNova(QuasicsDrivebaseCanIds.LEFT_LEADER_ID, MotorType.NEO),
        new ThriftyNova(QuasicsDrivebaseCanIds.RIGHT_LEADER_ID, MotorType.NEO),
        new Pigeon2(QuasicsDrivebaseCanIds.PIGEON2_CAN_ID));
  }

  /**
   * Constructor. Passes wrapped hardware up to the base class, and also sets up
   * the left/right follower motors on the drive base.
   *
   * @param config      drive base configuration
   * @param leftLeader  "leader" motor on the left side
   * @param rightLeader "leader" motor on the right side
   * @param rawGyro     the Pigeon2 ALU used on this drive base
   */
  protected ThriftyNovaDrivebase(DriveConfig config, ThriftyNova leftLeader,
      ThriftyNova rightLeader, Pigeon2 rawGyro) {
    super(config, new ThriftyNovaMotorControllerPlus(leftLeader),
        new ThriftyNovaMotorControllerPlus(rightLeader),
        new ThriftyEncoderWrapper(leftLeader, WHEEL_DIAMETER),
        new ThriftyEncoderWrapper(rightLeader, WHEEL_DIAMETER),
        new Pigeon2Wrapper(rawGyro));

    // Note: this should be redundant to work in the base class. (But it shouldn't
    // hurt.)
    leftLeader.setInverted(config.orientation().isLeftInverted());
    rightLeader.setInverted(config.orientation().isRightInverted());

    // Configure the other motors to follow their leader
    configureFollower(QuasicsDrivebaseCanIds.LEFT_FOLLOWER_ID, leftLeader);
    configureFollower(
        QuasicsDrivebaseCanIds.RIGHT_FOLLOWER_ID, rightLeader);
  }

  /**
   * Configures a motor (specified via CAN ID) to follow another motor.
   *
   * @param followerId CAN ID for the motor to be configured as a follower
   * @param leader     the motor that should serve as leader
   */
  private static void configureFollower(int followerId, ThriftyNova leader) {
    try (ThriftyNova follower = new ThriftyNova(followerId, MotorType.NEO)) {
      follower.follow(leader.getID());
      follower.setInverted(leader.getInversion());
    } catch (Exception e) { // Failures on ThriftyNova.close()
      throw new RuntimeException(e);
    }
  }
}
