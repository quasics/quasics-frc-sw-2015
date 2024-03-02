// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/** Add your docs here. */
public class Constants {
  public class AprilTags {
    /** April tags are square images, 8 1/8 inch to a side. */
    public static final Measure<Distance> APRIL_TAG_SIZE = Inches.of(8).plus(Inches.of(1.0 / 8.0));

    /**
     * Height to the bottom of the AprilTags (1, 2, 9, 10) on the Source positions.
     */
    public static final Measure<Distance> SOURCE_TAG_BOTTOM_HEIGHT = Feet.of(4).plus(Inches.of(1.0 / 8.0));

    /** Height to the bottom of the AprilTags (3, 4, 7, 8) on the Speakers. */
    public static final Measure<Distance> SPEAKER_TAG_BOTTOM_HEIGHT = Feet.of(4).plus(Inches.of(3 + 7.0 / 8.0));

    /** Height to the bottom of the AprilTags (5, 6) on the Amps. */
    public static final Measure<Distance> AMP_TAG_BOTTOM_HEIGHT = Feet.of(4).plus(Inches.of(1.0 / 8.0));

    /** Height to the bottom of the AprilTags (11-16) on the Stages. */
    public static final Measure<Distance> STAGE_TAG_BOTTOM_HEIGHT = Feet.of(3).plus(Inches.of(11.5));
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

  public static class DriveTeam {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
  }

  public static class CanBusIds {
    public static final int LEFT_CLIMBER_CAN_ID = 5;
    public static final int RIGHT_CLIMBER_CAN_ID = 6;
  }
}
