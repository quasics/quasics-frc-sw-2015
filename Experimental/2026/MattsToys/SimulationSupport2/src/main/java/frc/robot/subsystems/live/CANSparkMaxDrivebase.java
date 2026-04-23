// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static frc.robot.util.RevSupportFunctions.configureAsNotFollowing;
import static frc.robot.util.RevSupportFunctions.configureMotorToFollow;
import static frc.robot.util.RevSupportFunctions.configureSparkMaxEncoderForDistance;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.constants.robots.QuasicsSparkMaxConstants.QuasicsDrivebaseCanIds;
import frc.robot.hardware.actuators.SparkMaxMotorControllerPlus;
import frc.robot.hardware.sensors.Pigeon2Wrapper;
import frc.robot.hardware.sensors.SparkMaxEncoderWrapper;
import frc.robot.subsystems.bases.DrivebaseBase;
import frc.robot.util.config.DriveConfig;

/**
 * Sample implementation of the drivebase functionality, based on CAN-based
 * SparkMax controllers, using the "traditional" Quasics motor plan for such
 * robots.
 */
public class CANSparkMaxDrivebase extends DrivebaseBase {
  /**
   * Basic constructor.
   *
   * Note that this passes through to a more detailed constructor, so that we
   * can configure the leaders/followers without mucking around with unsafe
   * casts, etc.
   *
   * @param config drive base configuration
   */
  public CANSparkMaxDrivebase(DriveConfig config) {
    this(config,
        new SparkMax(
            config.motorIdMap().get(DriveConfig.MotorUnit.LeftLeader),
            MotorType.kBrushless),
        new SparkMax(
            config.motorIdMap().get(DriveConfig.MotorUnit.RightLeader),
            MotorType.kBrushless),
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
  protected CANSparkMaxDrivebase(DriveConfig config, SparkMax leftLeader,
      SparkMax rightLeader, Pigeon2 rawGyro) {
    super(config, new SparkMaxMotorControllerPlus(leftLeader),
        new SparkMaxMotorControllerPlus(rightLeader),
        new SparkMaxEncoderWrapper(leftLeader.getEncoder()),
        new SparkMaxEncoderWrapper(rightLeader.getEncoder()),
        new Pigeon2Wrapper(rawGyro), false);

    final SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    final SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();

    configureAsNotFollowing(leftLeaderConfig);
    configureAsNotFollowing(rightLeaderConfig);

    configureSparkMaxEncoderForDistance(
        leftLeaderConfig, config.wheelRadius(), config.gearing());
    configureSparkMaxEncoderForDistance(
        rightLeaderConfig, config.wheelRadius(), config.gearing());

    // We may need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward.
    rightLeaderConfig.inverted(config.orientation().isRightInverted());
    leftLeaderConfig.inverted(config.orientation().isLeftInverted());

    leftLeader.configure(leftLeaderConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // Configure the other motors (if we have them) to follow their respective
    // leaders.
    if (config.motorIdMap().containsKey(DriveConfig.MotorUnit.LeftFollower)) {
      configureMotorToFollow(
          config.motorIdMap().get(DriveConfig.MotorUnit.LeftFollower),
          leftLeader);
    }
    if (config.motorIdMap().containsKey(DriveConfig.MotorUnit.RightFollower)) {
      configureMotorToFollow(
          config.motorIdMap().get(DriveConfig.MotorUnit.RightFollower),
          rightLeader);
    }
  }
}
