package frc.robot.utils;

import java.io.File;
import java.io.IOException;
import java.util.Vector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AbstractDriveBase;

public class TrajectoryCommandGenerator {
  /**
   * The maximum voltage available to the motors while following the path. Should
   * be somewhat less than the nominal battery voltage (12V) to account for
   * "voltage sag" due to current draw.
   */
  public static final double MAX_VOLTAGE = 10;

  /// Drive profile settings, generally obtained from the FRC characterization
  /// tool (SysID, in 2022).
  public static class DriveProfileData {
    public DriveProfileData(double kS, double kV, double kA) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }

    public final double kS; // units::voltage::volt_t
    public final double kV; // VoltSecondsPerMeter
    public final double kA; // VoltSecondsSquaredPerMeter
  }

  /// PID configuration settings, to control error correction.
  public static class PIDConfig {
    public PIDConfig(double p) {
      this(p, 0, 0);
    }

    public PIDConfig(double p, double i, double d) {
      this.kP = p;
      this.kI = i;
      this.kD = d;
    }

    public final double kP; /// < proportional coefficient for PID (from the characterization tool)
    public final double kI; /// < integral coefficient for PID
    public final double kD; /// < derivative coefficient for PID (from the characterization tool)
  };

  /// Defines maximum speed/acceleration for the trajectory.
  ///
  /// Note that the default values are fairly low; we'll likely
  /// want use something larger for real code.
  public class SpeedProfile {
    public SpeedProfile() {
      this(0.5, 0.5);
    }

    public SpeedProfile(double maxV, double maxA) {
      maxVelocity = maxV;
      maxAcceleration = maxA;
    }

    final double maxVelocity; // MetersPerSecond
    final double maxAcceleration; // MetersPerSecondSquared
  };

  /// Values controlling for a RAMSETE follower in units of meters and
  /// seconds.
  ///
  /// If you don't understand these, don't worry too much about it. Just
  /// treat the defaults as "magic constants underpinning the universe",
  /// and you'll be fine. (Or you can dive into the docs, but the water
  /// gets pretty deep awfully fast.)
  private class RamseteConfig {
    // Tuning parameter (b > 0) for which larger values make
    // convergence more aggressive like a proportional term.
    double kRamseteB = 2;
    // Tuning parameter (0 < zeta < 1) for which larger values provide
    // more damping in response.
    double kRamseteZeta = 0.7;
  };

  private final AbstractDriveBase m_drive;
  private final DriveProfileData m_profileData;
  private final PIDConfig m_pidConfig;

  TrajectoryCommandGenerator(AbstractDriveBase driveBase, DriveProfileData profileData, PIDConfig pidConfig) {
    this.m_drive = driveBase;
    this.m_profileData = profileData;
    this.m_pidConfig = pidConfig;
  }

  public SequentialCommandGroup generateCommandFromPathWeaverFile(SpeedProfile speedProfile,
      String jsonFileName,
      boolean resetTelemetryAtStart) {
    File pathsDirectory = new File(Filesystem.getDeployDirectory(), "paths");
    File jsonFile = new File(pathsDirectory, jsonFileName);
    Trajectory trajectory;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(jsonFile.toPath());
      return GenerateCommandForTrajectory(m_drive, m_profileData, m_pidConfig, trajectory, resetTelemetryAtStart,
          new RamseteConfig());
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  public SequentialCommandGroup generateCommand(SpeedProfile speedProfile, Pose2d start,
      Vector<Translation2d> interiorWaypoints, Pose2d end, boolean resetTelemetryAtStart) {
    return GenerateCommandFromDiscreteSegments(
        m_drive,
        m_profileData,
        m_pidConfig,
        speedProfile,
        start,
        interiorWaypoints,
        end,
        resetTelemetryAtStart,
        new RamseteConfig());
  }

  private static SequentialCommandGroup GenerateCommandFromDiscreteSegments(
      AbstractDriveBase drive,
      DriveProfileData driveProfile,
      PIDConfig pidConfig,
      SpeedProfile speedProfile,
      Pose2d start,
      Vector<Translation2d> interiorWaypoints,
      Pose2d end,
      boolean resetTelemetryAtStart,
      RamseteConfig ramseteConfig) {
    final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(drive.getTrackWidth());

    final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
        driveProfile.kS,
        driveProfile.kV,
        driveProfile.kA);
    final DifferentialDriveVoltageConstraint voltageConstraints = new DifferentialDriveVoltageConstraint(feedForward,
        kDriveKinematics, MAX_VOLTAGE);

    final TrajectoryConfig config = new TrajectoryConfig(speedProfile.maxVelocity, speedProfile.maxAcceleration);
    config.setKinematics(kDriveKinematics);
    config.addConstraint(voltageConstraints);

    var trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

    return GenerateCommandForTrajectory(drive, driveProfile, pidConfig, trajectory,
        resetTelemetryAtStart, ramseteConfig);
  }

  static SequentialCommandGroup GenerateCommandForTrajectory(
      AbstractDriveBase drive,
      DriveProfileData driveProfile,
      PIDConfig pidConfig,
      Trajectory trajectory,
      boolean resetTelemetryAtStart,
      RamseteConfig ramseteConfig) {
    final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(drive.getTrackWidth());

    final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
        driveProfile.kS,
        driveProfile.kV,
        driveProfile.kA);

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory, () -> {
          return drive.getPose();
        },
        new RamseteController(ramseteConfig.kRamseteB, ramseteConfig.kRamseteZeta),
        feedForward,
        kDriveKinematics,
        () -> {
          return drive.getWheelSpeeds();
        },
        new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD),
        new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD),
        (var left, var right) -> {
          drive.tankDriveVolts(left, right);
        },
        drive);

    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              if (resetTelemetryAtStart) {
                System.out.println("Resetting robot odometry");
                drive.resetOdometry(trajectory.getInitialPose());
              }
            },
            drive),
        ramseteCommand,
        new InstantCommand(
            () -> {
              drive.tankDriveVolts(0, 0);
            },
            drive));
  }
}
