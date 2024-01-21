package frc.robot.utils;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecondSquared;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Voltage;

/**
 * Defines bot-specific configuration data for every one of Quasics' supported
 * instances in 2024.
 */
public interface RobotSettings {
  public enum MotorConfigModel {
    NoLeader, RearMotorsLeading, FrontMotorsLeading
  }

  /**
   * Enum class used to represent the characteristics (e.g., camera name,
   * PID/motor gain values, track width, etc.) that are specific to a given robot.
   *
   * For example, Sally is just a drive base and is thus much lighter than the
   * robots we generally put on the field, which means that the kS/kV values for
   * her tend to be smaller.
   */
  public enum Robot {
    /*
     * Supported values
     */
    Simulator(
        MotorConfigModel.NoLeader,
        /* Track Width (m) */
        0.381 * 2,
        /* Gear ratio */
        1.0,
        /* PID */
        1.3195, 0.0, 0.0, // WPI sample used 8.5, 0, 0
        /* Gains (WPI sample\ used 1, 3, 0) */
        Volts.of(0.013809), 1.9805, 0.19208,
        "photonvision", 9,
        40),
    // Xrp(MotorConfigModel.NoLeader, null, 9, 40),
    // Romi(MotorConfigModel.NoLeader, null, 9, 40);
    Sally(
        /* Track Width (m) */
        0.381 * 2, // TODO: Confirm track width for Sally
        /* Gear ratio */
        8.45,
        /* PID */
        0.29613, 0.0, 0.0,
        /* Gains */
        Volts.of(0.19529), 2.2329, 0.0),
    Mae(
        /* Track Width (m) */
        0.5588 /* 22in */, // TODO: Confirm track width for Mae
        /* Gear ratio */
        8.45,
        /* PID */
        0.001379, 0.0, 0.0, // TODO: Confirm kP for Mae, since it seems *really* low
        /*
         * Gains
         *
         * TODO: Confirm Gains for Mae, since they're very different from 2022
         * values. (Though we also changed the hardware significantly
         * post-season.)
         */
        Volts.of(0.13895), 1.3143, 0.1935),
    // Margaret()
    ;

    ////////////////////////////////////////////////////////
    // Drive base data
    public final MotorConfigModel motorConfigModel;
    public final double trackWidthMeters;
    public final double gearRatio;

    // PID obtained from SysId profiling
    public final double kP;
    public final double kI;
    public final double kD;
    // TODO: Add unit typing for gains data from SysId profiling.
    // Gains obtained from SysId profiling
    public final Measure<Voltage> kS;
    // public final Per<Voltage, Velocity<Distance>> kV;
    // public final Per<Voltage, Velocity<Velocity<Distance>>> kA;
    public final double kV;
    public final double kA;

    ////////////////////////////////////////////////////////
    // Vision subsystem data
    public final String cameraName;

    ////////////////////////////////////////////////////////
    // Lighting subsystem data
    private static final int DEFAULT_LIGHTING_PWM_PORT = 9;
    private static final int DEFAULT_NUM_LIGHTS = 9;
    public final int lightingPwmPort;
    public final int numLights;

    private Robot(
        double trackWidthMeters, double gearRatio,
        double kP, double kI, double kD,
        Measure<Voltage> kS, double kV, double kA) {
      this(MotorConfigModel.NoLeader, trackWidthMeters, gearRatio, kP, kI, kD, kS, kV, kA, "photonvision",
          DEFAULT_LIGHTING_PWM_PORT, DEFAULT_NUM_LIGHTS);
    }

    private Robot(MotorConfigModel motorConfigModel, double trackWidthMeters, double gearRatio, double kP,
        double kI, double kD,
        Measure<Voltage> kS, double kV, double kA,
        String cameraName, int lightingPort, int numLights) {
      this.motorConfigModel = motorConfigModel;
      this.trackWidthMeters = trackWidthMeters;
      this.gearRatio = gearRatio;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      this.cameraName = cameraName;
      this.numLights = numLights;
      this.lightingPwmPort = lightingPort;
    }
  }
}
