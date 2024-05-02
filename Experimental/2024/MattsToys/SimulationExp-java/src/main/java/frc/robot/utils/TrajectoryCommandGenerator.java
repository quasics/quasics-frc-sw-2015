// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class TrajectoryCommandGenerator {
  /**
   * The maximum voltage available to the motors while following the path. Should
   * be somewhat less than the nominal battery voltage (12V) to account for
   * "voltage sag" due to current draw.
   */
  public static final double MAX_VOLTAGE = 10;

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

  private final AbstractDrivebase m_drive;

  public TrajectoryCommandGenerator(AbstractDrivebase driveBase) {
    this.m_drive = driveBase;
  }

  public SequentialCommandGroup generateCommandFromPathWeaverFile(
      String jsonFileName, boolean resetTelemetryAtStart) {
    File pathsDirectory = new File(Filesystem.getDeployDirectory(), "paths");
    File jsonFile = new File(pathsDirectory, jsonFileName);
    Trajectory trajectory;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(jsonFile.toPath());
      return GenerateCommandForTrajectory(
          m_drive, trajectory, resetTelemetryAtStart, new RamseteConfig());
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  public SequentialCommandGroup generateCommand(TrajectoryConfig speedConfig, Pose2d start,
      List<Translation2d> interiorWaypoints, Pose2d end, boolean resetTelemetryAtStart) {
    return GenerateCommandFromDiscreteSegments(m_drive, speedConfig, start, interiorWaypoints, end,
        resetTelemetryAtStart, new RamseteConfig());
  }

  private static SequentialCommandGroup GenerateCommandFromDiscreteSegments(AbstractDrivebase drive,
      TrajectoryConfig trajectoryConfig, Pose2d start, List<Translation2d> interiorWaypoints,
      Pose2d end, boolean resetTelemetryAtStart, RamseteConfig ramseteConfig) {
    final DifferentialDriveKinematics kDriveKinematics = drive.getKinematics();

    final SimpleMotorFeedforward feedForward = drive.getMotorFeedforward();
    final DifferentialDriveVoltageConstraint voltageConstraints = new DifferentialDriveVoltageConstraint(feedForward,
        kDriveKinematics, MAX_VOLTAGE);

    TrajectoryConfig actualTrajectoryConfig = new TrajectoryConfig(
        trajectoryConfig.getMaxVelocity(), trajectoryConfig.getMaxAcceleration());
    actualTrajectoryConfig.setKinematics(kDriveKinematics);
    actualTrajectoryConfig.addConstraint(voltageConstraints);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        start, interiorWaypoints, end, actualTrajectoryConfig);

    return GenerateCommandForTrajectory(drive, trajectory, resetTelemetryAtStart, ramseteConfig);
  }

  static SequentialCommandGroup GenerateCommandForTrajectory(AbstractDrivebase drive,
      Trajectory trajectory, boolean resetTelemetryAtStart, RamseteConfig ramseteConfig) {
    final SimpleMotorFeedforward feedForward = drive.getMotorFeedforward();
    final Supplier<Pose2d> poseSupplier = () -> {
      return drive.getPose();
    };
    final Supplier<DifferentialDriveWheelSpeeds> wheelSpeedSupplier = () -> {
      return drive.getWheelSpeeds();
    };
    final BiConsumer<Double, Double> motorVoltageConsumer = (var left, var right) -> {
      drive.setMotorVoltages(left, right);
    };
    final double kP = drive.getKP();
    final double kI = drive.getKI();
    final double kD = drive.getKD();
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, poseSupplier,
        new RamseteController(ramseteConfig.kRamseteB, ramseteConfig.kRamseteZeta), feedForward,
        drive.getKinematics(), wheelSpeedSupplier, new PIDController(kP, kI, kD),
        new PIDController(kP, kI, kD), motorVoltageConsumer, (Subsystem) drive);

    return new SequentialCommandGroup(new InstantCommand(() -> {
      if (resetTelemetryAtStart) {
        System.out.println("Resetting robot odometry");
        drive.resetOdometry(trajectory.getInitialPose());
      }
    }, drive), ramseteCommand, new InstantCommand(() -> {
      drive.stop();
    }, drive));
  }
}
