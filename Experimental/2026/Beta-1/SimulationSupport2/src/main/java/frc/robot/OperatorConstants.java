package frc.robot;

public interface OperatorConstants {
  public static final int DRIVER_JOYSTICK_ID = 0;

  // Rate limits for accelerating the drive base: require a ramp-up of (no less
  // than) 1/3 sec from 0 to 100% (or vice versa).
  public static final double MAX_SLEW_RATE = 3;

  public static final double DEADBAND_THRESHOLD = 0.05;
}
