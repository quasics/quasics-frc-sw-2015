#pragma once

#include <frc/Filesystem.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include <iostream>
#include <vector>

// In "Common2021"
#include "CommonDriveSubsystem.h"

class TrajectoryCommandGenerator {
  //
  // Type definitions, referenced above.
 public:
  typedef decltype(1_V * 1_s / 1_m) VoltSecondsPerMeter;
  typedef decltype(1_V * 1_s * 1_s / 1_m) VoltSecondsSquaredPerMeter;
  typedef decltype(1_mps) MetersPerSecond;
  typedef decltype(1_mps_sq) MetersPerSecondSquared;

  /// Drive profile settings, generally obtained from the frc-characterization
  /// tool.
  struct DriveProfileData {
    units::voltage::volt_t kS;
    VoltSecondsPerMeter kV;
    VoltSecondsSquaredPerMeter kA;
  };

  /// PID configuration settings, to control error correction.
  struct PIDConfig {
    double kP;      ///< proportional coefficient for PID (from the
                    ///< frc-characterization tool)
    double kI = 0;  ///< integral coefficient for PID (default to unused)
    double kD = 0;  ///< derivative coefficient for PID (default to unused)
  };

  /// Defines maximum speed/acceleration for the trajectory.
  ///
  /// Note that the default values are fairly low; you'll likely
  /// want use something larger for real code.
  struct SpeedProfile {
    MetersPerSecond maxVelocity = 0.5_mps;
    MetersPerSecondSquared maxAcceleration = 0.5_mps_sq;
  };

  /// Values controlling for a RAMSETE follower in units of meters and
  /// seconds.
  ///
  /// If you don't understand these, don't worry too much about it.  Just
  /// treat the defaults as "magic constants underpinning the universe",
  /// and you'll be fine.  (Or you can dive into the docs, but the water
  /// gets pretty deep awfully fast.)
  struct RamseteConfig {
    // Tuning parameter (b > 0) for which larger values make
    // convergence more aggressive like a proportional term.
    double kRamseteB = 2;
    // Tuning parameter (0 < zeta < 1) for which larger values provide
    // more damping in response.
    double kRamseteZeta = 0.7;
  };

 public:
  /**
   * Constructor.
   * 
   * @param drive  the drive subsystem to be used for moving the robot
   * @param profileData  drive profile data, describing its performance
   * @param pidConfig  PID configuration values, used for error-correction
   * 
   * @see #DriveProfileData
   * @see #PIDConfig
   */
  TrajectoryCommandGenerator(CommonDriveSubsystem* drive,
                             const DriveProfileData& profileData,
                             const PIDConfig& pidConfig)
      : m_drive(drive), m_profileData(profileData), m_pidConfig(pidConfig) {
  }

  /**
   * Generates a sample command to follow the specified trajectory.
   *
   * @param speedProfile
   *     maximum velocity/acceleration constraints to be observed
   * @param start
   *     the starting pose of the robot, relative to drive telemetry data
   * @param interiorWaypoints
   *     the points on the field through which the robot should pass while
   *     passing from "start" to "end"
   * @param end
   *     the ending pose of the robot, relative to drive telemetry data
   * @param resetTelemetryAtStart
   *     if true, the command will (at its initiation) reset the drive
   *     telemetry, allowing it to start following the trajectory using
   *     its current position and orientation as the origin point (0,0).
   *     Otherwise, it will use the previously-established origin as a
   *     starting point (and first drive back to that).
   */
  frc2::SequentialCommandGroup* GenerateCommand(
      const SpeedProfile speedProfile, const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart) {
    return GenerateCommandFromDiscreteSegments(
        m_drive, m_profileData, m_pidConfig, speedProfile, start,
        interiorWaypoints, end, resetTelemetryAtStart, m_ramseteConfig);
  }

  /**
   * Generates a sample command to follow a trajectory stored in a PathWeaver
   * JSON file.
   *
   * @param jsonFileName
   *     name of the file holding the JSON data for the trajectory, as exported
   *     by the PathWeaver tool.  (This is assumed to be stored in the normal
   *     "deploy" folder for the target system, which will either be
   *     project-relative when running with the simulator, or in the stock
   *     location on a Rio, if running on real hardware.)
   * @param resetTelemetryAtStart
   *     if true, the command will (at its initiation) reset the drive
   *     telemetry, allowing it to start following the trajectory using
   *     its current position and orientation as the origin point (0,0).
   *     Otherwise, it will use the previously-established origin as a
   *     starting point (and first drive back to that).
   */
  frc2::SequentialCommandGroup* GenerateCommandFromPathWeaverFile(
      const std::string jsonFileName, bool resetTelemetryAtStart) {
    wpi::SmallString<256>
        deployDirectory;  // Not thrilled about the limit to "SmallString",
                          // given emulation, but it's what's supported.
    frc::filesystem::GetDeployDirectory(deployDirectory);
    wpi::sys::path::append(deployDirectory, "paths");
    wpi::sys::path::append(deployDirectory, jsonFileName);

    frc::Trajectory trajectory =
        frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

    return GenerateCommandForTrajectory(m_drive, m_profileData, m_pidConfig,
                                        trajectory, resetTelemetryAtStart,
                                        m_ramseteConfig);
  }

 private:
  // Having a static version helps to prevent accidental captures
  // of "this" in lambdas: if the generator is a local variable,
  // that's going to make the code go "boom" when you try to use
  // the command later....
  static frc2::SequentialCommandGroup* GenerateCommandFromDiscreteSegments(
      CommonDriveSubsystem* const drive, const DriveProfileData profileData,
      const PIDConfig pidConfig, const SpeedProfile speedProfile,
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart,
      const RamseteConfig ramseteConfig);

  static frc2::SequentialCommandGroup* GenerateCommandForTrajectory(
      CommonDriveSubsystem* const drive, const DriveProfileData profileData,
      const PIDConfig pidConfig, frc::Trajectory trajectory,
      bool resetTelemetryAtStart, const RamseteConfig ramseteConfig);

 private:
  CommonDriveSubsystem* m_drive;
  const DriveProfileData m_profileData;
  const PIDConfig m_pidConfig;

  // I'm not even going to expose this for overriding for now.  Just treat it
  // as magic numbers, based on the recommendation from FIRST and WPILib.
  const RamseteConfig m_ramseteConfig;
};

inline frc2::SequentialCommandGroup*
TrajectoryCommandGenerator::GenerateCommandFromDiscreteSegments(
    CommonDriveSubsystem* const drive, const DriveProfileData profileData,
    const PIDConfig pidConfig, const SpeedProfile speedProfile,
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart,
    const RamseteConfig ramseteConfig) {
  const frc::DifferentialDriveKinematics kDriveKinematics{
      drive->GetTrackWidth()};

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      profileData.kS, profileData.kV, profileData.kA);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);

  frc::TrajectoryConfig config(speedProfile.maxVelocity,
                               speedProfile.maxAcceleration);
  config.SetKinematics(kDriveKinematics);
  config.AddConstraint(voltageConstraints);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  return GenerateCommandForTrajectory(drive, profileData, pidConfig, trajectory,
                                      resetTelemetryAtStart, ramseteConfig);
}

inline frc2::SequentialCommandGroup*
TrajectoryCommandGenerator::GenerateCommandForTrajectory(
    CommonDriveSubsystem* const drive, const DriveProfileData profileData,
    const PIDConfig pidConfig, frc::Trajectory trajectory,
    bool resetTelemetryAtStart, RamseteConfig ramseteConfig) {
  const frc::DifferentialDriveKinematics kDriveKinematics{
      drive->GetTrackWidth()};
  frc::SimpleMotorFeedforward<units::meter> feedForward(
      profileData.kS, profileData.kV, profileData.kA);
  frc2::RamseteCommand ramseteCommand(
      trajectory, [drive]() { return drive->GetPose(); },
      frc::RamseteController{ramseteConfig.kRamseteB,
                             ramseteConfig.kRamseteZeta},
      feedForward, kDriveKinematics,
      [drive]() { return drive->GetWheelSpeeds(); },
      frc2::PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD),
      frc2::PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD),
      [drive](auto left, auto right) { drive->TankDriveVolts(left, right); },
      {drive});

  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand(
          [drive, resetTelemetryAtStart, trajectory] {
            if (resetTelemetryAtStart) {
              std::cout << "Resetting robot odometry" << std::endl;
              drive->ResetOdometry(trajectory.InitialPose());
            }
          },
          {drive}),
      std::move(ramseteCommand),
      frc2::InstantCommand([drive] { drive->TankDriveVolts(0_V, 0_V); },
                           {drive}));
}
