package frc.robot.util.config;

import edu.wpi.first.units.measure.Distance;

/**
 * Drive base configuration data.
 *
 * @param driveType   drive type (simulated, CAN-based SparkMax)
 * @param wheelRadius radius of the drive base wheels
 * @param trackWidth  maximum width between drive base wheels
 * @param gearing     gearing between motor and wheel axel (>=1)
 * @param orientation orientation of the drivebase's motors
 * @param leftPid     PID configuration for the drivebase's left motors
 * @param rightPid    PID configuration for the drivebase's right motors
 * @param feedForward feedforward configuration for the drivebase
 */
public record DriveConfig(DriveType driveType, Distance wheelRadius,
    Distance trackWidth, double gearing, DriveOrientation orientation,
    PIDConfig leftPid, PIDConfig rightPid, DriveFeedForwardConfig feedForward) {
  /**
   * Convenience constructor, using a single set of PID values for both left
   * and right.
   *
   * @param driveType   drive type (simulated, CAN-based SparkMax)
   * @param wheelRadius radius of the drive base wheels
   * @param trackWidth  maximum width between drive base wheels
   * @param gearing     gearing between motor and wheel axel (>=1)
   * @param orientation orientation of the drivebase's motors
   * @param commonPid   shared PID configuration for the drivebase
   * @param feedForward feedforward configuration for the drivebase
   */
  public DriveConfig(DriveType driveType, Distance wheelRadius,
      Distance trackWidth, double gearing, DriveOrientation orientation,
      PIDConfig commonPid, DriveFeedForwardConfig feedForward) {
    this(driveType, wheelRadius, trackWidth, gearing, orientation, commonPid,
        commonPid, feedForward);
  }
}