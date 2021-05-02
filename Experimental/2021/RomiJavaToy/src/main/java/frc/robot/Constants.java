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
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double NORMAL_MODE_SPEED_LIMT = 0.75;
    public static final double TURBO_MODE_SPEED_LIMT = 1.0;
    public static final double TURTLE_MODE_SPEED_LIMT = 0.75;

    public static final double THROTTLE_DEADBAND_LIMIT = 0.06;
    public static final double SPEED_SCALING_DEADBAND_LIMIT = 0.25;

    public static final int DRIVER_JOYSTICK_PORT = 0;

    public static final int GAMESIR_LEFT_HORIZONTAL = 0;
    public static final int GAMESIR_LEFT_VERTICAL = 1;
    public static final int GAMESIR_LEFT_TRIGGER = 2; // Range is [0,1]
    public static final int GAMESIR_RIGHT_TRIGGER = 3; // Range is [0,1]
    public static final int GAMESIR_RIGHT_HORIZONTAL = 4;
    public static final int GAMESIR_RIGHT_VERTICAL = 5;
}
