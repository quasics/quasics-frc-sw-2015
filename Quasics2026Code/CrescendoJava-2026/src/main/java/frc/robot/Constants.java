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

      public static final int LEFT_CLIMBER_ID = 5;
      public static final int RIGHT_CLIMBER_ID = 6;

      public static final int AMP_MOTOR_ID = 0; // update when id is known
      public static final int TRANSITION_MOTOR_ID =
          7; // update when we know port
      public static final int INTAKE_MOTOR_ID = 8;

      public static final int LEFT_SHOOTER_ID = 9;
      public static final int RIGHT_SHOOTER_ID = 10;
    }

    public static final int PIGEON2_CAN_ID = 1;
  }

  public static class RobotSpeedScaling {
    public static final double TURBO_MODE_SPEED_SCALING = 0.9;
    public static final double TURTLE_MODE_SPEED_SCALING = 0.3;
    public static final double NORMAL_MODE_SPEED_SCALING = 0.6;
  }
  public static class AutonomousStartingPositions {
    public static final String leftOfSpeaker = "Left of speaker"; // 1B
    public static final String inFrontOfSpeaker = "In front of speaker"; // 2
    public static final String rightOfSpeaker = "Right of speaker"; // 3A
    public static final String farField = "Far field"; // 3B
  } // public static class AutonomousStartingPositions

  public static class AutonomousSelectedOperation {
    public static final String doNothing = "Do nothing";
    public static final String GTFO = "GTFO";
    public static final String score1 = "Score 1 piece";
    public static final String score1GTFO = "Score 1 piece, GTFO";
    public static final String score2 = "Score 2 piece";
    public static final String score2GTFO = "Score 2 piece, GTFO";
    public static final String score3 = "Score 3 piece";
    public static final String score3GTFO = "Score 3 piece, GTFO";
    public static final String score4 = "Score 4 piece";
  } // public static class AutonomousSelectedOperation

  public static class AutonomousScore2Options {
    public static final String none = "None";
    public static final String rightOfSpeakerAllianceNote =
        "Score right of speaker (alliance note)";
    public static final String rightOfSpeakerCenterNote =
        "Score right of speaker (center note)";
  } // public static class AutonomousScore2Options

  public static class AutonomousScore3Options {
    public static final String none = "None";
    public static final String amp = "Score in amp";
    public static final String leftOfSpeaker = "Score left of speaker";
    public static final String inFrontOfSpeakerAmpNote =
        "Score in front of speaker (amp note)";
    public static final String inFrontOfSpeakerStageNote =
        "Score in front of speaker (stage note)";
    public static final String inFrontOfSpeakerCenterNote =
        "Score in front of speaker (center note)";
    public static final String rightOfSpeaker = "Score right of speaker";
  } // public static class AutonomousScore3Options
  public static class PathWeaverConstantsSally {
    /*
    public static final double kS = 0.19529;
    public static final double kV = 2.2329;
    public static final double kA = 0.36638;
    public static final double kP = 0.29613;
    public static final double kI = 0;
    public static final double kD = 0;
    */

    public static final double kS = 0;
    public static final double kV = 0.18741;
    public static final double kA = 0.044339;
    public static final double kP = 0.11025;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class PathWeaverConstantsMargaret { // average of left and right
                                                    // characterizations
    public static final double kS = (0.017001 + 0.015565) / 2;
    public static final double kV = (0.1979 + 0.19042) / 2;
    public static final double kA = (0.031501 + 0.030128) / 2;
    public static final double kP = (0.042133 + 0.041392) / 2;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static class PwmIDs {
    public static final int LedControl = 0;
  }
}
