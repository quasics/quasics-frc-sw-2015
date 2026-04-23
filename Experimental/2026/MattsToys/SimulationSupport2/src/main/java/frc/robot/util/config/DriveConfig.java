package frc.robot.util.config;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.robots.QuasicsSparkMaxConstants;
import frc.robot.constants.robots.QuasicsThriftyNovaConstants;

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
 * @param motorIdMap  mapping of drive motors to their corresponding IDs (CAN,
 *                    PWM, etc.)
 */
public record DriveConfig(DriveType driveType, Distance wheelRadius,
    Distance trackWidth, double gearing, DriveOrientation orientation,
    PIDConfig leftPid, PIDConfig rightPid, DriveFeedForwardConfig feedForward, Map<MotorUnit, Integer> motorIdMap) {
  public enum MotorUnit {
    LeftLeader, RightLeader, LeftFollower, RightFollower
  }

  private static final Map<MotorUnit, Integer> DEFAULT_NOVA_MOTOR_ID_MAP = Map.of(
      MotorUnit.LeftLeader, QuasicsThriftyNovaConstants.QuasicsDrivebaseCanIds.LEFT_LEADER_ID,
      MotorUnit.RightLeader, QuasicsThriftyNovaConstants.QuasicsDrivebaseCanIds.RIGHT_LEADER_ID,
      MotorUnit.LeftFollower, QuasicsThriftyNovaConstants.QuasicsDrivebaseCanIds.LEFT_FOLLOWER_ID,
      MotorUnit.RightFollower, QuasicsThriftyNovaConstants.QuasicsDrivebaseCanIds.RIGHT_FOLLOWER_ID);

  private static final Map<MotorUnit, Integer> DEFAULT_SPARK_MOTOR_ID_MAP = Map.of(
      MotorUnit.LeftLeader, QuasicsSparkMaxConstants.QuasicsDrivebaseCanIds.LEFT_LEADER_ID,
      MotorUnit.RightLeader, QuasicsSparkMaxConstants.QuasicsDrivebaseCanIds.RIGHT_LEADER_ID,
      MotorUnit.LeftFollower, QuasicsSparkMaxConstants.QuasicsDrivebaseCanIds.LEFT_FOLLOWER_ID,
      MotorUnit.RightFollower, QuasicsSparkMaxConstants.QuasicsDrivebaseCanIds.RIGHT_FOLLOWER_ID);

  static Map<MotorUnit, Integer> getDefaultMotorIdMap(DriveType driveType) {
    return switch (driveType) {
      case Simulated -> null;
      case CanSparkMax -> DEFAULT_SPARK_MOTOR_ID_MAP;
      case ThriftyNova -> DEFAULT_NOVA_MOTOR_ID_MAP;
    };
  }

  /**
   * Convenience constructor, using a default (canonical) motor ID mapping.
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
      PIDConfig leftPid, PIDConfig rightPid, DriveFeedForwardConfig feedForward) {
    this(driveType, wheelRadius, trackWidth, gearing, orientation, leftPid,
        rightPid, feedForward, getDefaultMotorIdMap(driveType));
  }

  /**
   * Convenience constructor, using a single set of PID values for both left
   * and right, and a default (canonical) motor ID mapping.
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
        commonPid, feedForward, getDefaultMotorIdMap(driveType));
  }
}