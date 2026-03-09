// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** Radius of the drive base wheels. */
  public static final Distance WHEEL_RADIUS = Inches.of(3);

  /** Gearing ratio for kitbots (which also happens to match what we're using). */
  public static final KitbotGearing GEARING = KitbotGearing.k8p45;

  /** Gearing ratio for our robots this year. */
  public static final double DRIVEBASE_GEAR_RATIO = GEARING.value;

  public static final LinearVelocity MAX_LINEAR_DRIVE_SPEED = MetersPerSecond.of(3);
  public static final AngularVelocity MAX_ROTATIONAL_SPEED = RadiansPerSecond.of(2);

  /**
   * Maximum length used in the lighting subsystem for the addressable LED strip.
   */
  public static final int LIGHTING_TOTAL_LENGTH = 80;

  // TODO: Calculate https://www.chiefdelphi.com/t/coefficient-of-friction/467778
  // TODO: mass, moi, CoF, current draw, track width for auto

  public static class PwmPortIds {
    public static final int SIMULATED_LEFT_MOTOR_CHANNEL = 0;
    public static final int SIMULATED_RIGHT_MOTOR_CHANNEL = 1;

    /** PWM port ID for addressable lighting control. */
    public static final int LIGHTING_ID = 2;
  }

  public static class CanBusIds {
    /** CAN ID for the Pigeon2 hardware. */
    public static final int PIGEON2_CAN_ID = 1;

    /**
     * CAN IDs for SparkMax motors used on any of the robots this year (generally
     * Lizzie or Sally).
     */
    public static class SparkMaxIds {
      // Note: Drive base motor IDs are based on those Quasics has used over the
      // last couple of years.
      public static final int LEFT_LEADER_ID = 2;
      public static final int LEFT_FOLLOWER_ID = 1;
      public static final int RIGHT_LEADER_ID = 4;
      public static final int RIGHT_FOLLOWER_ID = 3;
      public static final int INTAKE_ROLLERS_ID = 5;
      public static final int RIGHT_INTAKE_DEPLOYMENT_ID = 6;
      public static final int LEFT_INTAKE_DEPLOYMENT_ID = 7;
      public static final int INDEXER_ID = 8;
      public static final int KICKER_ID = 9;
      public static final int HOOD_ID = 10;
      public static final int SHOOTER_ID = 11;
      public static final int CLIMBER_ID = 12;
    }

    /**
     * CAN IDs for ThriftyNova motors used this year. (This should just be the drive
     * motors in Lizzie.)
     */
    public static class ThriftyNovaIds {
      public static final int LEFT_LEADER_ID = 2;
      public static final int LEFT_FOLLOWER_ID = 1;
      public static final int RIGHT_LEADER_ID = 4;
      public static final int RIGHT_FOLLOWER_ID = 3;
    }
  }

  public static class DriveteamConstants {
    public static final int DRIVER_JOYSTICK_ID = 0;
    public static final int OPERATOR_JOYSTICK_ID = 1;
  }

  /**
   * PID values for the flywheel on the shooter.
   */
  public static class FlywheelPIDConstants {
    public static final double kV = 0.11676;
    public static final double kP = 0.17735;
  }

  public static class Tolerances {
    public static final Angle ANGLETOLERANCE = Degrees.of(8);
  }

  public static class Ratios {
    public static final double ENCODERTOHOODRATIO = 9;
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

  /**
   * Helpful field calculations based off of Rebuilt numbers.
   */
  public class RebuiltFieldData {
    public static final Distance FIELD_LENGTH = Inches.of(651.22);
    public static final Distance FIELD_WIDTH = Inches.of(317.69);
    public static final Distance FIELD_LENGTH_CENTER = Inches.of(325.61);
    public static final Distance FIELD_WIDTH_CENTER = Inches.of(158.84);
    public static final Distance ALLIANCE_WALL_TO_HUB_CENTER = Inches.of(182.11);
  }

  /**
   * Settings for normal/turtle/turbo speed modes. (These will be applied as a
   * scaling factor to the raw inputs from the driver joysticks.)
   */
  public static class RobotSpeedScaling {
    /** Turtle mode speed scaling factor. */
    public static final double TURTLE_SPEED_SCALING = 0.1;
    /** Normal mode speed scaling factor. */
    public static final double NORMAL_SPEED_SCALING = 0.2;
    /** Turbo mode speed scaling factor. */
    public static final double TURBO_SPEED_SCALING = 0.7;
  }

  /**
   * Types of drivebases that we "know about" in the code. (Assumption is that all
   * of the motors in the drivebase are of a common type.)
   */
  // TODO: Likely want configurable settings other than just this (track width,
  // etc): See RobotSettings.java in Reefscape code
  public static enum DrivebaseMotors {
    Unknown,
    SparkMax,
    ThriftyNova
  }

  // TODO: Make a com.pathplanner.lib.config.RobotConfig per robot we want to run
  // auto on
}
