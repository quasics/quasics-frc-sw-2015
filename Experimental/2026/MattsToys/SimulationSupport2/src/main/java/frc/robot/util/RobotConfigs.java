// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import java.util.Collections;
import java.util.List;

/**
 * Defines various configuration data for robot subsystems.
 */
public interface RobotConfigs {
  /** Invalid CAN ID. */
  public static final int INVALID_CAN_ID = -1;

  /**
   * Location of something in terms of the robot's center (in terms of the robot
   * coordinate system).
   *
   * This is currently used for the camera(s), but could easily be used for
   * other things as needed. (Note: I could've used a Translation3D for this,
   * but felt that "position" was more readable.)
   *
   * @param x distance along the X axis (front (+)/back (-)) from center
   * @param y distance along the Y axis (left (+)/right (-)) from center
   * @param z distance along the Z axis (up (+)/down (-)) from center
   *
   * @see <a
   *      href=
   *      "https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system">Robot
   *      coordinate system</a>
   */
  public static record Position(Distance x, Distance y, Distance z) {}

  /**
   * Describes a camera's orientiation relative to the robot.
   *
   * @param pitch The counterclockwise rotation angle around the Y axis (pitch).
   * @param roll  The counterclockwise rotation angle around the X axis (roll).
   * @param yaw   The counterclockwise rotation angle around the Z axis (yaw).
   */
  public static record Orientation(Angle pitch, Angle roll, Angle yaw) {}

  /**
   * Defines the image-related characteristics of a camera on the robot.
   *
   * Note that some of these characteristics would only be used (directly) in
   * the code for simulation purposes.
   *
   * @param width  camera field width (in pixels)
   * @param height camera field height (in pixels)
   * @param fov    field of view (e.g., 100 degrees)
   * @param fps    frames per second produced by the video stream
   */
  public static record Imaging(int width, int height, Angle fov, double fps) {}

  /**
   * Describes the camera's configuration.
   *
   * @param name        name of the camera (as exposed through PhotonVision)
   * @param pos         camera position, relative to the center of the robot
   * @param orientation angling/rotation of the camera (relative to robot
   *                    centerline, flat)
   * @param imaging     characteristics of the camera's image feed
   */
  public static record CameraConfig(String name, Position pos,
                                    Orientation orientation, Imaging imaging) {}

  /**
   * PID configuration settings.
   *
   * @param kP proportional constant
   * @param kI integral constant
   * @param kD derivitive constant
   */
  public static record PIDConfig(double kP, double kI, double kD) {
    /**
     * Overloaded ctor for kP-only configs.
     *
     * @param kP proportional constant
     */
    public PIDConfig(double kP) { this(kP, 0.0, 0.0); }
  }

  /**
   * Elevator Feed forward settings.
   *
   * @param kS static gain
   * @param kG gravity gain
   * @param kV kV, in V/(m/s)
   * @param kA kA, in V/(m/s^2)
   */
  public static record ElevatorFeedForwardConfig(Voltage kS, Voltage kG,
                                                 double kV, double kA) {
    /**
     * Overloaded constructor.
     *
     * @param kS static gain, in V
     * @param kG gravity gain, in V
     * @param kV kV, in V/(m/s)
     * @param kA kA, in V/(m/s^2)
     */
    public ElevatorFeedForwardConfig(double kS, double kG, double kV,
                                     double kA) {
      this(Volts.of(kS), Volts.of(kG), kV, kA);
    }
  }

  /**
   * Configuration data for an elevator.
   *
   * @param pid         PID configuration settings for the elevator's motors
   * @param feedForward feedforward data for the elevator
   */
  public static record ElevatorConfig(PIDConfig pid,
                                      ElevatorFeedForwardConfig feedForward) {}

  /**
   * Simple (i.e., not for elevators) feedforward data.
   *
   * @param kS kS, in V
   * @param kV kV, in V/(m/s); must be > 0
   * @param kA kA, in V/(m/s^2)
   */
  public static record SimpleFeedForwardConfig(Voltage kS, Voltage kV,
                                               double kA) {
    /**
     * Overloaded constructor.
     *
     * @param kV kV, in V/(m/s); must be > 0
     * @param kA kA, in V/(m/s^2)
     */
    public SimpleFeedForwardConfig(Voltage kV, double kA) {
      this(Volts.of(0), kV, kA);
    }

    /**
     * Overloaded constructor.
     *
     * @param kV kV, in V/(m/s); must be > 0
     * @param kA kA, in V/(m/s^2)
     */
    public SimpleFeedForwardConfig(double kV, double kA) {
      this(Volts.of(0), Volts.of(kV), kA);
    }

    /**
     * Overloaded constructor.
     *
     * @param kV kV, in V/(m/s); must be > 0
     * @param kA kA, in V/(m/s^2)
     */
    public SimpleFeedForwardConfig(Voltage kS, double kV, double kA) {
      this(kS, Volts.of(kV), kA);
    }
  }

  /**
   * Drive Feed forward settings.
   *
   * @param linear  linear feedforward settings
   * @param angular angular (rotational) feedforward settings
   */
  public static record DriveFeedForwardConfig(SimpleFeedForwardConfig linear,
                                              SimpleFeedForwardConfig angular) {
    /**
     * Overloaded constructor.
     * 
     * @para, ksLinear  linear feedforward kS value
     * @param kvLinear  linear feedforward kV value
     * @param kaLinear  linear feedforward kA value
     * @param kvAngular angular (rotational) feedforward kV value
     * @param kaAngular angular (rotational) feedforward kA value
     */
    public DriveFeedForwardConfig(Voltage ksLinear, Voltage kvLinear, double kaLinear,
                                  Voltage kvAngular, double kaAngular) {
      this(new SimpleFeedForwardConfig(ksLinear, kvLinear, kaLinear),
           new SimpleFeedForwardConfig(kvAngular, kaAngular));
    }

    /**
     * Overloaded constructor, taking pairs of (kV, kA) as discrete values
     *
     * @param kvLinear  linear feedforward kV value
     * @param kaLinear  linear feedforward kA value
     * @param kvAngular angular (rotational) feedforward kV value
     * @param kaAngular angular (rotational) feedforward kA value
     */
    public DriveFeedForwardConfig(Voltage kvLinear, double kaLinear,
                                  Voltage kvAngular, double kaAngular) {
      this(new SimpleFeedForwardConfig(kvLinear, kaLinear),
           new SimpleFeedForwardConfig(kvAngular, kaAngular));
    }
  }

  /**
   * Drive base configuration data.
   *
   * @param wheelRadius radius of the drive base wheels
   * @param trackWidth  maximum width between drive base wheels
   * @param gearing     gearing between motor and wheel axel (>=1)
   * @param leftPid     PID configuration for the drivebase's left motors
   * @param rightPid    PID configuration for the drivebase's right motors
   * @param feedForward feedforward configuration for the drivebase
   */
  public static record DriveConfig(Distance wheelRadius, Distance trackWidth,
                                   double gearing, PIDConfig leftPid,
                                   PIDConfig rightPid,
                                   DriveFeedForwardConfig feedForward) {
    /**
     * Convenience constructor, using a single set of PID values for both left
     * and right.
     *
     * @param wheelRadius radius of the drive base wheels
     * @param trackWidth  maximum width between drive base wheels
     * @param gearing     gearing between motor and wheel axel (>=1)
     * @param commonPid   shared PID configuration for the drivebase
     * @param feedForward feedforward configuration for the drivebase
     */
    public DriveConfig(Distance wheelRadius, Distance trackWidth,
                       double gearing, PIDConfig commonPid,
                       DriveFeedForwardConfig feedForward) {
      this(wheelRadius, trackWidth, gearing, commonPid, commonPid, feedForward);
    }
  }

  /**
   * Lighting subsystem configuration data.
   *
   * @param pwmPort     the PWM port driving the LED strip
   * @param stripLength the length (in pixels/cells) of the LED strip
   * @param subViews    list of subviews for the strip (segments following the
   *                    "main set")
   *
   * @see frc.robot.subsystems.live.Lighting
   * @see frc.robot.subsystems.live.LightingBuffer
   */
  public static record LightingConfig(int pwmPort, int stripLength,
                                      List<Integer> subViews) {
    /**
     * Constructor (with sanity checking).
     *
     * @param pwmPort     the PWM port driving the LED strip
     * @param stripLength the length (in pixels/cells) of the LED strip
     * @param subViews    list of subviews for the strip (segments following the
     *                    "main set")
     */
    public LightingConfig {
      if (subViews != null) {
        final int subViewTotalSize =
            subViews.stream().mapToInt(Integer::intValue).sum();
        if (subViewTotalSize > stripLength) {
          throw new IllegalArgumentException(
              "Sub-view size (" + subViewTotalSize +
              ") exceeds strip length (" + stripLength + ")");
        }
      }
    }

    /**
     * Convenience constructor.
     *
     * @param pwmPort     the PWM port driving the LED strip
     * @param stripLength the length (in pixels/cells) of the LED strip
     */
    public LightingConfig(int pwmPort, int stripLength) {
      this(pwmPort, stripLength, null);
    }
  }

  /**
   * Arm Feed forward settings.
   *
   * @param kS static gain
   * @param kG gravity gain
   * @param kV kV, in V/(m/s)
   * @param kA kA, in V/(m/s^2)
   */
  public static record ArmFeedForwardConfig(Voltage kS, Voltage kG, double kV,
                                            double kA) {
    /**
     * Overloaded constructor (no kA).
     *
     * @param kS static gain, in V
     * @param kG gravity gain, in V
     * @param kV kV, in V/(m/s)
     */
    public ArmFeedForwardConfig(Voltage kS, Voltage kG, double kV) {
      this(kS, kG, kV, 0);
    }

    /**
     * Overloaded constructor (all unitless).
     *
     * @param kS static gain, in V
     * @param kG gravity gain, in V
     * @param kV kV, in V/(m/s)
     * @param kA kA, in V/(m/s^2)
     */
    public ArmFeedForwardConfig(double kS, double kG, double kV, double kA) {
      this(Volts.of(kS), Volts.of(kG), kV, kA);
    }

    /**
     * Overloaded constructor (all unitless, no kA).
     *
     * @param kS static gain, in V
     * @param kG gravity gain, in V
     * @param kV kV, in V/(m/s)
     */
    public ArmFeedForwardConfig(double kS, double kG, double kV) {
      this(Volts.of(kS), Volts.of(kG), kV, 0);
    }
  }

  /**
   * Single-joint arm configuration settings.
   *
   * @param pid         PID settings for the arm
   * @param feedForward feedforward settings for the arm
   */
  public static record ArmConfig(PIDConfig pid,
                                 ArmFeedForwardConfig feedForward) {}

  /**
   * CANdle configuration settings.
   *
   * @param canId CAN ID for the device; if negative, the device should be
   *              simulated
   */
  public static record CandleConfig(int canId) {
    /**
     * Determines if the CANdle is simulated.
     *
     * @return true iff the CANdle is simulated (based on the CAN ID)
     */
    public boolean simulated() { return canId < 0; }
  }

  /**
   * Collective robot configuration data.
   *
   * @param drive    drive base configuration (may be null)
   * @param cameras  list of camera configurations (may be null)
   * @param elevator elevator configuration (may be null)
   * @param lighting lighting configuration (may be null)
   * @param arm      arm configuration (may be null)
   * @param candle   CANdle configuration (may be null)
   */
  public static record RobotConfig(DriveConfig drive,
                                   List<CameraConfig> cameras,
                                   ElevatorConfig elevator, ArmConfig arm,
                                   LightingConfig lighting,
                                   CandleConfig candle) {
    /**
     * Utility constructor fo a single-camera robot.
     *
     * @param drive    drive base configuration (may be null)
     * @param camera   camera configuration (may be null)
     * @param elevator elevator configuration (may be null)
     * @param lighting lighting configuration (may be null)
     * @param arm      arm configuration (may be null)
     * @param candle   CANdle configuration (may be null)
     */
    RobotConfig(DriveConfig drive, CameraConfig camera, ElevatorConfig elevator,
                ArmConfig arm, LightingConfig lighting, CandleConfig candle) {
      this(drive, Collections.singletonList(camera), elevator, arm, lighting,
           candle);
    }

    /**
     * Determines if we have drive configuration data.
     *
     * @return true iff the configuration includes data for the drivebase
     */
    public boolean hasDrive() { return drive != null; }

    /**
     * Determines if we have camera configuration data.
     *
     * @return true iff the configuration includes data for the camera
     */
    public boolean hasCamera() {
      return (cameras() != null) && !cameras.isEmpty();
    }

    /**
     * Determines if we have elevator configuration data.
     *
     * @return true iff the configuration includes data for the elevator
     */
    public boolean hasElevator() { return elevator != null; }

    /**
     * Determines if we have arm configuration data.
     *
     * @return true iff the configuration includes data for the arm
     */
    public boolean hasArm() { return arm != null; }

    /**
     * Determines if we have lighting configuration data.
     *
     * @return true iff the configuration includes data for the lighting
     */
    public boolean hasLighting() { return lighting != null; }

    /**
     * Determines if we have CANdle configuration data.
     *
     * @return true iff the configuration includes data for the CANdle
     */
    public boolean hasCandle() { return candle != null; }
  }

  public static final DriveConfig NO_DRIVE = null;
  public static final CameraConfig NO_CAMERA = null;
  public static final ElevatorConfig NO_ELEVATOR = null;
  public static final LightingConfig NO_LIGHTING = null;
  public static final ArmConfig NO_ARM = null;
  public static final CandleConfig NO_CANDLE = null;
}
