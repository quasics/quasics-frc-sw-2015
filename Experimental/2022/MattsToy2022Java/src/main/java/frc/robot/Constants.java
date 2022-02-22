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

  public static final double INCHES_PER_METER = 39.3701;

  public static final double TRACK_WIDTH_METERS_ROMI = 0.165;

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

  /** Constants for speed limits/scaling under various driving modes. */
  public static final class SpeedLimits {
    /** Scaling factor to be applied in "turtle" mode. */
    public static final double MAX_SPEED_TURTLE = 0.50;
    /** Scaling factor to be applied in "normal" mode. */
    public static final double MAX_SPEED_NORMAL = 0.65;
    /** Scaling factor to be applied in "turbo" mode. */
    public static final double MAX_SPEED_TURBO = 0.80;
  }

  public static final class OperatorInterface {
    /**
     * The joystick to be used for driving (i.e., its index in the joystick set).
     */
    public static final int DRIVER_JOYSTICK = 0;

    /**
     * Constants associated with the controls on a Logitech F310 gamepad.
     */
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

    /**
     * Constants associated with the controls on a GameSir G4 Pro Bluetooth Game
     * Controller.
     */
    public static final class GameSirPro {
      // Axes - Used with the "getRawAxis()" function to access the data for the
      // individual sticks on the controller (e.g., for "tank drive" coding).
      //
      // Note that the left and right triggers aren't treated as buttons: they
      // report to the driver's station software as if they're joysticks (with a
      // range of [0.0, 1.0], unlike regular joysticks).
      public static final int LEFT_X_AXIS = 0;
      public static final int LEFT_Y_AXIS = 1;
      public static final int RIGHT_X_AXIS = 4;
      public static final int RIGHT_Y_AXIS = 5;

      public static final int A_BUTTON = 1;
      public static final int B_BUTTON = 2;
      public static final int X_BUTTON = 3;
      public static final int Y_BUTTON = 4;
      public static final int LEFT_SHOULDER_BUTTON = 5;
      public static final int RIGHT_SHOULDER_BUTTON = 6;
      public static final int G_BUTTON = 7;
      public static final int S_BUTTON = 8;
      public static final int LEFT_STICK_PRESS = 9;
      public static final int RIGHT_STICK_PRESS = 10;
    }
  }

  /**
   * Constants to be used in configuring the lighting subsystem.
   */
  public static final class Lighting {
    /** PWM port to which the LED strip is connected. */
    public static final int PWM_PORT = 7;

    /** Number of LEDs in the lighting strip. */
    public static final int NUM_LIGHTS = 60;
  }

  /**
   * Constants associated with "dead zones" on joysticks.
   * 
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  public static final class Deadbands {
    /**
     * Dead band value to be used for deciding when the driver joysticks are at
     * neutral.
     */
    public static final double DRIVING = 0.055;
  }
}
