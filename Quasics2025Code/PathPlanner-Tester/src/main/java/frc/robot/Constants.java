// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner
 * classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class LogitechGamePad {
    // Axes - Used with the "getRawAxis()" function to access the data for the
    // individual sticks on the controller (e.g., for "tank drive" coding).
    //
    // Note that the left and right triggers aren't treated as buttons: they
    // report to the driver's station software as if they're joysticks (with a
    // range of [0.0, 1.0], unlike regular joysticks).
    public static final int LeftXAxis = 0;
    public static final int LeftYAxis = 1;
    public static final int LeftTriggerAxis = 2;
    public static final int RightTriggerAxis = 3;
    public static final int RightXAxis = 2;
    public static final int RightYAxis = 3;

    // Buttons
    public static final int XButton = 1;
    public static final int AButton = 2;
    public static final int BButton = 3;
    public static final int YButton = 4;
    public static final int LeftShoulder = 5;
    public static final int RightShoulder = 6;
    public static final int LeftTrigger = 7;
    public static final int RightTrigger = 8;
    public static final int BackButton = 9;
    public static final int StartButton = 10;
    public static final int LeftStickPress = 11;
    public static final int RightStickPress = 12;
  }

  public static class DriveTeam {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
  }

  public static class CanBusIds {
    public static class SparkMaxIds {
      // Note: Drive base motor IDs are based on those Quasics has used over the
      // last couple of years.
      public static final int LEFT_LEADER_ID = 2;
      public static final int LEFT_FOLLOWER_ID = 1;
      public static final int RIGHT_LEADER_ID = 4;
      public static final int RIGHT_FOLLOWER_ID = 3;
    }
    public static final int PIGEON2_CAN_ID = 1;
  }

  public static class SimulationPorts {
    public static final int LEFT_FRONT_DRIVE_PWM_ID = 1;
    public static final int LEFT_REAR_DRIVE_PWM_ID = 2;
    public static final int RIGHT_FRONT_DRIVE_PWM_ID = 3;
    public static final int RIGHT_REAR_DRIVE_PWM_ID = 4;

    public static final int LEFT_DRIVE_ENCODER_PORT_A = 0;
    public static final int LEFT_DRIVE_ENCODER_PORT_B = 1;
    public static final int RIGHT_DRIVE_ENCODER_PORT_A = 2;
    public static final int RIGHT_DRIVE_ENCODER_PORT_B = 3;
  }
}
