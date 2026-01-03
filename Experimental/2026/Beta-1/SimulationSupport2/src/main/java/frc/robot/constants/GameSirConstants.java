// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants associated with the controls on a GameSir G4 Pro Bluetooth Game
 * Controller.
 */
public class GameSirConstants {
  // Axes - Used with the "getRawAxis()" function to access the data for the
  // individual sticks on the controller (e.g., for "tank drive" coding).
  //
  // Note that the left and right triggers aren't treated as buttons: they
  // report to the driver's station software as if they're joysticks (with a
  // range of [0.0, 1.0], unlike regular joysticks).
  public class Axes {
    /** Left joystick X axis. (Right is positive.) */
    public static final int LEFT_X = 0;
    /** Left joystick Y axis. (Forward is negative.) */
    public static final int LEFT_Y = 1;
    /** Left trigger axis. (Range is [0,1].) */
    public static final int LEFT_TRIGGER = 2;
    /** Right trigger axis. (Range is [0,1].) */
    public static final int RIGHT_TRIGGER = 3;
    /** Right joystick X axis. (Right is positive.) */
    public static final int RIGHT_X = 4;
    /** Right joystick Y axis. (Forward is negative.) */
    public static final int RIGHT_Y = 5;
  }

  public class Buttons {
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LEFT_SHOULDER = 5;
    public static final int RIGHT_SHOULDER = 6;
    public static final int G = 7;
    public static final int S = 8;
    public static final int LEFT_STICK_PRESS = 9;
    public static final int RIGHT_STICK_PRESS = 10;
  }
}
