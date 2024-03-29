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
    public static final class DIO {
        public static final int CLIMBER_UPPER_LIMIT_SWITCH_ID = 1;
        public static final int CLIMBER_LOWER_LIMIT_SWITCH_ID = 2;
    }

    public static final class MotorIds {
        public static final class SparkMax {
            public static final int RIGHT_CLIMBER_MOTOR_ID = 6;
            public static final int LEFT_CLIMBER_MOTOR_ID = 7;
        }
    }
}
