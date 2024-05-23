// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
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
    public static class SparkMax {
      // Note: Drive base motor IDs are based on those Quasics has used over the last
      // couple of years.
      public static final int LEFT_LEADER_ID = 2;
      public static final int RIGHT_LEADER_ID = 4;


      public static final int LEFT_CLIMBER_ID = 5;
      public static final int RIGHT_CLIMBER_ID = 6;

      public static final int INTAKE_MOTOR_ID = 8;

      public static final int LEFT_SHOOTER_ID = 9;
      public static final int RIGHT_SHOOTER_ID = 10;
    }

    public static final int PIGEON2_CAN_ID = 1;
  }

}
