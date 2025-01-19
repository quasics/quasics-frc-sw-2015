package frc.robot;

public class Constants {

  public static class DriveTeam {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;

    public static final double DRIVER_DEADBAND = 0.05;
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
    public static final int AButton = 1;
    public static final int BButton = 2;
    public static final int XButton = 3;
    public static final int YButton = 4;
    public static final int LeftShoulder = 5;
    public static final int RightShoulder = 6;
    public static final int BackButton = 7;
    public static final int StartButton = 8;
    public static final int LeftStickPress = 9;
    public static final int RightStickPress = 10;
  }

}
