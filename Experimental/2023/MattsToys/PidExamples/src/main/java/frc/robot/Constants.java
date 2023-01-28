// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class MotorIds {
    public static final class SparkMax {
      public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
      public static final int LEFT_REAR_DRIVE_MOTOR_ID = 2;
      public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
      public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
    }
  }

  public static final double DRIVE_BASE_GEAR_RATIO_SALLY = 8.45; // 2022 robot

  public static final double INCHES_PER_METER = 39.3701;

  public static final double WHEEL_DIAMETER_INCHES = 6.0;
  public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES / INCHES_PER_METER;

  public static final double MAX_SPEED_METERS_PER_SEC = 5.0;

}
