// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

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

//

public final class Constants {
  public static class SallyConstants {
    public static final double TRACK_WIDTH = 0.5588;
  }

  /**
   * Defines constants for the different controls on a Logitech gamepad when used
   * in "Dualshock" mode.
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
    /** ID for the "X" button. */
    public static final int XButton = 1;
    /** ID for the "A" button. */
    public static final int AButton = 2;
    /** ID for the "B" button. */
    public static final int BButton = 3;
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
   * Defines constants for the different controls on a Logitech gamepad, when used
   * in "F310" mode.
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
    /** ID for the "X" button. */
    public static final int XButton = 1;
    /** ID for the "A" button. */
    public static final int AButton = 2;
    /** ID for the "B" button. */
    public static final int BButton = 3;
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

  public static class DriveTeam {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
  }

  public static class CanBusIds {
    public static class SparkMaxIds {
      // Note: Drive base motor IDs are based on those Quasics has used over the last
      // couple of years.
      public static final int LEFT_LEADER_ID = 2;
      public static final int LEFT_FOLLOWER_ID = 1;
      public static final int RIGHT_LEADER_ID = 4;
      public static final int RIGHT_FOLLOWER_ID = 3;

      // TODO: update these
      public static final int LEFT_CLIMBER_ID = 5;
      public static final int RIGHT_CLIMBER_ID = 6;

      public static final int FOLLOWER_ELEVATOR_ID = 7;
      public static final int LEADER_ELEVATOR_ID = 8;

      public static final int ARM_PIVOT_ID = 9;

      public static final int ARM_ROLLER_ID = 10;
    }

    public static final int PIGEON2_CAN_ID = 1;
  }

  public static class RobotSpeedScaling {
    public static final double TURBO_MODE_SPEED_SCALING = 0.9;
    public static final double TURTLE_MODE_SPEED_SCALING = 0.3;
    public static final double NORMAL_MODE_SPEED_SCALING = 0.6;
  }

  public static class AutonomousStartingPositions {
    public static final String VERY_TOP = "very top (4)";
    public static final String TOP = "top (1)";
    public static final String MIDDLE = "middle (2)";
    public static final String BOTTOM = "bottom (3)";
    public static final String VERY_BOTTOM = "very bottom (5)";
  } // public static class AutonomousStartingPositions

  public static class AutonomousSelectedOperation {
    public static final String DO_NOTHING = "Do nothing";
    public static final String GTFO = "GTFO";
    public static final String GO_TO_REEF = "Go to reef";
    public static final String GO_TO_REEF_DR = "Go to reef DR";
    public static final String GRAB_ALGAE_FROM_REEF = "Grab algae from reef";
    public static final String SCORE_CORAL_IN_REEF = "Score coral in reef";
    public static final String SCORE_ALGAE_REEF_BARGE = "Score algae from reef into barge";
    public static final String SCORE_ALGAE_REEF_PROCESSOR = "Score algae from reef into processor";
    public static final String GRAB_ALGAE_FROM_FIELD = "Grab algae from field";
    public static final String SCORE_ALGAE_FIELD_PROCESSOR = "Score algae from field into processor";
    public static final String SCORE_ALGAE_FIELD_BARGE = "Score algae from field into barge";

  } // public static class AutonomousSelectedOperation

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DesiredEncoderValues {
    public static final double ARM_DOWN = 0; // TODO: needs testing
    public static final double ARM_UP = 0.265; // TODO: needs updated once the value can be tested
    public static final Angle ARM_DOWN_IN_RADIANS = Radians.of(0);
    public static final Angle ARM_UP_IN_RADIANS = Radians.of(1.5708);
  }

  public static class ArmPIDConstants {
    // TODO: PID/feedforward constants for the arm need to be tuned. (Using zeros
    // right now.)
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
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
