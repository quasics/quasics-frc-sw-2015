// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // TODO(mjh): Confirm this value for Sally....
  public static final double TRACK_WIDTH_INCHES_SALLY = 22.0;

  // TODO(mjh): Confirm this value for Mae....
  public static final double TRACK_WIDTH_INCHES_MAE = 47.134344149315914763; // 2021 robot's width, from in-code
  // constants (which are in meters)

  // TODO(mjh): Confirm this for Nike
  public static final double TRACK_WIDTH_INCHES_NIKE = TRACK_WIDTH_INCHES_MAE;

  public static final double WHEEL_DIAMETER_INCHES = 6.0;

  /// Gear ratio used for the 2020/2021 robots.
  public static final double DRIVE_BASE_GEAR_RATIO_2021 = 10.71;
  public static final double DRIVE_BASE_GEAR_RATIO_2022 = 8.45;
  public static final double DRIVE_BASE_GEAR_RATIO = DRIVE_BASE_GEAR_RATIO_2022;

  public static final int PIGEON2_CAN_ID = 1;

  public static final class MotorIds {
    public static final class SparkMax {
      public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
      public static final int LEFT_REAR_DRIVE_MOTOR_ID = 2;
      public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
      public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;

      public static final int SHOOTER_MOTOR_ID = 5;
    }
  }

  public static final class OperatorInterface {
    public static final int DRIVER_JOYSTICK = 0;

    public static final class LogitechGamePad {
      // Note: these values were derived from one of the Logitech-branded controllers
      // on 22Jan2022. But it looks like there may be differences between apparently
      // identical devices.... :-(

      // Axes - Used with the "getRawAxis()" function to access the data for the
      // individual sticks on the controller (e.g., for "tank drive" coding).
      //
      // Note that the left and right triggers aren't treated as buttons: they
      // report to the driver's station software as if they're joysticks (with a
      // range of [0.0, 1.0], unlike regular joysticks).
      public static final int LEFT_X_AXIS = 0;
      public static final int LEFT_Y_AXIS = 1;
      public static final int RIGHT_X_AXIS = 2;
      public static final int RIGHT_Y_AXIS = 3;

      // Buttons
      public static final int A_BUTTON = 2; // Labeled "2" on some controllers
      public static final int B_BUTTON = 3; // Labeled "3" on some controllers
      public static final int X_BUTTON = 1; // Labeled "1" on some controllers
      public static final int Y_BUTTON = 4; // Labeled "4" on some controllers
      public static final int LEFT_SHOULDER = 5;
      public static final int RIGHT_SHOULDER = 6;
      public static final int LEFT_TRIGGER = 7;
      public static final int RIGHT_TRIGGER = 8;
      public static final int BACK_BUTTON = 9;
      public static final int START_BUTTON = 10;
      public static final int LEFT_STICK_PRESS = 11;
      public static final int RIGHT_STICK_PRESS = 12;
    }
  }

  public static final class Lighting {
    public static final int PWM_PORT = 7;
    public static final int NUM_LIGHTS = 60;
  }

  public static final class Deadbands {
    public static final double DRIVING = 0.055;
  }
}
