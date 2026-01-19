// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.robots;

/**
 * Defines some constants for use with Quasics' Thrifty Nova-based robots.
 */
public class QuasicsThriftyNovaConstants {

  /** Common CAN IDs for Quasics' robots. */
  public static class QuasicsDrivebaseCanIds {
    /** CAN ID for a Pigeon2 ALU. */
    public static final int PIGEON2_CAN_ID = 1;

    /** CAN ID for the "leading" motor on the drive base's left side. */
    public static final int LEFT_LEADER_ID = 2;
    /** CAN ID for the "following" motor on the drive base's left side. */
    public static final int LEFT_FOLLOWER_ID = 1;
    /** CAN ID for the "leading" motor on the drive base's right side. */
    public static final int RIGHT_LEADER_ID = 4;
    /** CAN ID for the "following" motor on the drive base's right side. */
    public static final int RIGHT_FOLLOWER_ID = 3;
  }

}
