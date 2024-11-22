package frc.robot.subsystems.simulations;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class Constants {
  public static final int LEFT_DRIVE_PWM_ID = 0;
  public static final int RIGHT_DRIVE_PWM_ID = 1;

  public static final int LEFT_DRIVE_ENCODER_PORT_A = 0;
  public static final int LEFT_DRIVE_ENCODER_PORT_B = 1;
  public static final int RIGHT_DRIVE_ENCODER_PORT_A = 2;
  public static final int RIGHT_DRIVE_ENCODER_PORT_B = 3;

  public static final Distance kWheelRadiusMeters = Units.Meters.of(0.0508);
  public static final Distance kRobotTrackWidthMeters = Units.Meters.of(0.381 * 2);
  public static final int kEncoderResolutionTicksPerRevolution = -4096;
}
