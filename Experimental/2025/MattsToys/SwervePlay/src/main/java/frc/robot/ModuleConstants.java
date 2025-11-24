package frc.robot;

public class ModuleConstants {
    public final static boolean kDriveInverted = true;
    public final static boolean kSteerInverted = true;

    public final static double kSteerP = 1.0;
    public final static double kSteerI = 0.0;
    public final static double kSteerD = 0.0;
    public final static double kMinSteerOutput = -1.0;
    public final static double kMaxSteerOutput = +1.0;
    public final static double kSteerGearRatio = 40.0;
    public final static double kDriveVelocityConversionFactor = 40.0;

    public final static int kFrontLeftDriveCANId = 1;
    public final static int kFrontLeftSteerCANId = 2;
    public final static double kFrontLeftOffset = 0.0; // absolute encoder offset to define 0 degrees (in rotations)
    public final static int kRearLeftDriveCANId = 3;
    public final static int kRearLeftSteerCANId = 4;
    public final static double kRearLeftOffset = 0.0; // absolute encoder offset to define 0 degrees (in rotations)
    public final static int kFrontRightDriveCANId = 5;
    public final static int kFrontRightSteerCANId = 6;
    public final static double kFrontRightOffset = 0.0; // absolute encoder offset to define 0 degrees (in rotations)
    public final static int kRearRightDriveCANId = 7;
    public final static int kRearRightSteerCANId = 8;
    public final static double kRearRightOffset = 0.0; // absolute encoder offset to define 0 degrees (in rotations)
}
