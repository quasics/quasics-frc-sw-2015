// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Defines some constants for the robot.
 */
public class Constants {
  /**
   * Constants applied to drive team controls.
   */
  public static class DriveTeam {
    /** ID for the driver's controller. */
    public static final int DRIVER_JOYSTICK_ID = 0;
    /** ID for the operator's controller. */
    public static final int OPERATOR_JOYSTICK_ID = 1;

    /** Deadband setting for the driver's joysticks. */
    public static final double DRIVER_DEADBAND = 0.05;
  }

  /**
   * Defines constants for the different controls on a Logitech gamepad.
   * 
   * Note that the left and right triggers aren't treated as buttons: they
   * report to the driver's station software as if they're single-axis joysticks
   * (with a range of [0.0, 1.0], unlike regular joysticks).
   * 
   * Note: these values assume that the switch on the bottom of the Logitech
   * controller is in the "D" position, causing it to enumerate as a Logitech
   * Dualshock controller. In this mode, the right joystick X/Y axes are 2 and 3,
   * respectively, and the left and right triggers show up as *buttons* 7 and
   * 8.
   */
  public static class LogitechDualshock {
    //
    // Axes - Used with the "getRawAxis()" function to access the data for the
    // individual sticks on the controller (e.g., for "tank drive" coding).
    //
    /** Left joystick X axis. */
    public static final int LeftXAxis = 0;
    /** Left joystick Y axis. */
    public static final int LeftYAxis = 1;
    /** Right joystick X axis. */
    public static final int RightXAxis = 2;
    /** Right joystick Y axis. */
    public static final int RightYAxis = 3;

    //
    // Buttons
    //
    /** ID for the "A" button. */
    public static final int AButton = 1;
    /** ID for the "B" button. */
    public static final int BButton = 2;
    /** ID for the "X" button. */
    public static final int XButton = 3;
    /** ID for the "Y" button. */
    public static final int YButton = 4;
    /** ID for the left shoulder button. */
    public static final int LeftShoulder = 5;
    /** ID for the right shoulder button. */
    public static final int RightShoulder = 6;
    /** Left trigger's X axis. */
    public static final int LeftTrigger = 7;
    /** Right trigger's X axis. */
    public static final int RightTrigger = 8;
    /** ID for the back button. */
    public static final int BackButton = 9;
    /** ID for the start button. */
    public static final int StartButton = 10;
    /** ID for the button clicked by pressing on the left joystick. */
    public static final int LeftStickPress = 11;
    /** ID for the button clicked by pressing on the right joystick. */
    public static final int RightStickPress = 12;
  }

  /**
   * Defines constants for the different controls on a Logitech gamepad.
   * 
   * Note that the left and right triggers aren't treated as buttons:
   * 
   * Note: these values assume that the switch on the bottom of the Logitech
   * controller is in the "X" position, causing it to enumerate as a Logitech
   * Gamepad F310. In this mode, the left and right triggers on the front
   * enumerate as single-axis joysticks 2 and 3 with a range of [0.0, 1.0], unlike
   * regular joysticks.
   */
  public static class LogitechGamePadF310 {
    //
    // Axes - Used with the "getRawAxis()" function to access the data for the
    // individual sticks on the controller (e.g., for "tank drive" coding).
    //
    /** Left joystick X axis. */
    public static final int LeftXAxis = 0;
    /** Left joystick Y axis. */
    public static final int LeftYAxis = 1;
    /** Right joystick X axis. */
    public static final int RightXAxis = 4;
    /** Right joystick Y axis. */
    public static final int RightYAxis = 5;
    /** Left trigger X axis. */
    public static final int LeftTriggerAxis = 2;
    /** Right trigger X axis. */
    public static final int RightTriggerAxis = 3;

    //
    // Buttons
    //
    /** ID for the "A" button. */
    public static final int AButton = 1;
    /** ID for the "B" button. */
    public static final int BButton = 2;
    /** ID for the "X" button. */
    public static final int XButton = 3;
    /** ID for the "Y" button. */
    public static final int YButton = 4;
    /** ID for the left shoulder button. */
    public static final int LeftShoulder = 5;
    /** ID for the right shoulder button. */
    public static final int RightShoulder = 6;
    /** ID for the "Back" button. */
    public static final int BackButton = 7;
    /** ID for the "Start" button. */
    public static final int StartButton = 8;
    /** ID for the button clicked by pressing on the left joystick. */
    public static final int LeftStickPress = 9;
    /** ID for the button clicked by pressing on the right joystick. */
    public static final int RightStickPress = 10;
  }
}
