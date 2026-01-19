// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.robots;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Defines some constants used annually by Quasics for their SparkMax-based
 * robots.
 */
public class QuasicsSparkMaxConstants {
  public static class Driving {
    // Rate limits for accelerating the drive base: require a ramp-up of (no
    // less than) 1/3 sec from 0 to 100% (or vice versa).
    public static final double MAX_SLEW_RATE = 3;
  }

  /** Common CAN IDs for Quasics' robots. */
  public static class QuasicsDrivebaseCanIds {
    /** CAN ID for a Pigeon2 ALU. */
    public static final int PIGEON2_CAN_ID = 1;
    /** CAN ID for the "leading" motor on the drive base's left side. */
    public static final int LEFT_LEADER_ID = 2;
    /** CAN ID for the "following" motor on the drive base's left side. */
    public static final int LEFT_FOLLOWER_ID = 1;
    /** CAN ID for the "leading" motor on the drive base's right side. */
    public static final int RIGHT_LEADER_ID = 4;
    /** CAN ID for the "following" motor on the drive base's right side. */
    public static final int RIGHT_FOLLOWER_ID = 3;
  }

  /** Other CAN IDs for Quasics' 2025 robot. */
  public static class OtherCanIds {
    /** Following motor for the elevator. */
    public static final int FOLLOWER_ELEVATOR_ID = 7;
    /** Leading motor for the elevator. */
    public static final int LEADER_ELEVATOR_ID = 8;
    /** CAN ID for the single-joint arm. */
    public static final int ARM_LEADER_ID = 9;

    /** CAN ID for the CANdle. */
    public static final int CANDLE_ID = 1;
  }

  /** Digital IO IDs. */
  public static class DioIds {
    /** ID for the elevator's "top" limit switch. */
    public static final int ELEVATOR_LIMIT_SWITCH_UP = 0;
    /** ID for the elevator's "bottom" limit switch. */
    public static final int ELEVATOR_LIMIT_SWITCH_DOWN = 1;

    public static final int LAST_NONSIM_DIO = ELEVATOR_LIMIT_SWITCH_DOWN;
  }

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
   * Defines constants for the different controls on a Logitech gamepad when
   * used in "Dualshock" mode.
   *
   * Note: these values assume that the switch on the bottom of the Logitech
   * controller is in the "D" position, causing it to enumerate as a Logitech
   * Dualshock controller. In this mode, the right joystick X/Y axes are 2 and
   * 3, respectively, and the left and right triggers show up as *buttons* 7 and
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
   * Defines constants for the different controls on a Logitech gamepad, when
   * used in "F310" mode.
   *
   * Note: these values assume that the switch on the bottom of the Logitech
   * controller is in the "X" position, causing it to enumerate as a Logitech
   * Gamepad F310. In this mode, the left and right triggers on the front
   * enumerate as single-axis joysticks 2 and 3 with a range of [0.0, 1.0],
   * unlike regular joysticks.
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

  /** Constants associated with the vision class. */
  public static class VisionConstants {
    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be
     * discarded
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    /** Pose ambiguity compensation. */
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    /** Pose ambiguity compensation. */
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    /** Distance (in meters) past which we'll ignore a target. */
    public static final double NOISY_DISTANCE_METERS = 2.5;
    /** Weight associated with distance. */
    public static final double DISTANCE_WEIGHT = 7;
    /** Confidence compensation weight. */
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in meters and radians, then meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        new Matrix<N3, N1>(Nat.N3(), Nat.N1(),
            new double[] {
                // if these numbers are less than one, multiplying will do bad
                // things
                1, // x
                1, // y
                1 * Math.PI // theta
            });

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision less. This matrix is in the form
     * [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        new Matrix<N3, N1>(Nat.N3(), Nat.N1(),
            new double[] {// if these numbers are less than one, multiplying
                          // will do bad things
                .1, // x
                .1, // y
                .1});
  }

  /** Field-related constants for Reefscape. */
  public static class FieldConstants {
    /** Center line is ~8.77m from the blue alliance wall. */
    final static public Distance CENTER_LINE = Meters.of(8.77);

    /**
     * Starting line for blue alliance is ~7.58m from the blue alliance wall.
     */
    final static public Distance BLUE_STARTING_LINE = Meters.of(7.58);

    /** Starting line for red alliance is ~9.96m from the red alliance wall. */
    final static public Distance RED_STARTING_LINE = Meters.of(9.96);

    /** How far away from the starting line we can shoot. */
    final static public Distance FAR_SHOOTING_RANGE = Meters.of(0.25);

    /** How close in to the starting line we can shoot. */
    final static public Distance NEAR_SHOOTING_RANGE = Meters.of(0.0);

    /** Outer range for when we're on the blue alliance. */
    final static public Distance BLUE_FAR_SHOOTING_RANGE =
        BLUE_STARTING_LINE.minus(FAR_SHOOTING_RANGE);

    /** Inner range for when we're on the blue alliance. */
    final static public Distance BLUE_NEAR_SHOOTING_RANGE =
        BLUE_STARTING_LINE.minus(NEAR_SHOOTING_RANGE);

    /** Outer range for when we're on the red alliance. */
    final static public Distance RED_FAR_SHOOTING_RANGE =
        RED_STARTING_LINE.plus(FAR_SHOOTING_RANGE);

    /** Inner range for when we're on the red alliance. */
    final static public Distance RED_NEAR_SHOOTING_RANGE =
        RED_STARTING_LINE.plus(NEAR_SHOOTING_RANGE);

    /** Ideal angle for shooting at the barge from the red alliance. */
    final static public Angle RED_BASE_SHOOTING_ANGLE = Degrees.of(180);
    /** Ideal angle for shooting at the barge from the blue alliance. */
    final static public Angle BLUE_BASE_SHOOTING_ANGLE = Degrees.of(0);
    /** Tolerance for our shooting angle. */
    final static public Angle SHOOTING_ANGLE_TOLERANCE = Degrees.of(4);

    /** Outer range on the angle for when we're on the blue alliance. */
    final static public Angle BLUE_FAR_SHOOTING_ANGLE =
        BLUE_BASE_SHOOTING_ANGLE.minus(SHOOTING_ANGLE_TOLERANCE);

    /** Inner range on the angle for when we're on the blue alliance. */
    final static public Angle BLUE_NEAR_SHOOTING_ANGLE =
        BLUE_BASE_SHOOTING_ANGLE.plus(SHOOTING_ANGLE_TOLERANCE);

    /** Outer range on the angle for when we're on the blue alliance. */
    final static public Angle RED_FAR_SHOOTING_ANGLE =
        RED_BASE_SHOOTING_ANGLE.minus(SHOOTING_ANGLE_TOLERANCE);

    /** Inner range on the angle for when we're on the blue alliance. */
    final static public Angle RED_NEAR_SHOOTING_ANGLE =
        RED_BASE_SHOOTING_ANGLE.plus(SHOOTING_ANGLE_TOLERANCE);
  }
}
