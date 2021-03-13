#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
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
  TrajectoryCommandGenerator(CommonDriveSubsystem* drive,
                             const DriveProfileData& profileData,
                             const PIDConfig& pidConfig,
                             const SpeedProfile& speedProfile)
      : m_drive(drive),
        m_profileData(profileData),
        m_pidConfig(pidConfig),
        m_speedProfile(speedProfile) {
  }

  TrajectoryCommandGenerator(CommonDriveSubsystem* drive,
                             units::voltage::volt_t kS, VoltSecondsPerMeter kV,
                             VoltSecondsSquaredPerMeter kA, double kP,
                             double kI, double kD, MetersPerSecond maxVelocity,
                             MetersPerSecondSquared maxAcceleration)
      : m_drive(drive),
        m_profileData{kS, kV, kA},
        m_pidConfig{kP, kI, kD},
        m_speedProfile{maxVelocity, maxAcceleration} {
  }

  void SetSpeedProfile(const SpeedProfile& speedProfile) {
    m_speedProfile = speedProfile;
  }

  frc2::SequentialCommandGroup* GenerateCommand(
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart) {
    return GenerateCommandImpl(m_drive, m_profileData, m_pidConfig,
                               m_speedProfile, m_ramseteConfig, start,
                               interiorWaypoints, end, resetTelemetryAtStart);
  }

 private:
  // Having a static version helps to prevent accidental captures
  // of "this" in lambdas: if the generator is a local variable,
  // that's going to make the code go "boom" when you try to use
  // the command later....
  static frc2::SequentialCommandGroup* GenerateCommandImpl(
      CommonDriveSubsystem* const drive, const DriveProfileData profileData,
      const PIDConfig pidConfig, const SpeedProfile speedProfile,
      const RamseteConfig ramseteConfig, const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart);

 private:
  CommonDriveSubsystem* m_drive;
  const DriveProfileData m_profileData;
  const PIDConfig m_pidConfig;
  SpeedProfile m_speedProfile;

  // I'm not even going to expose this for overriding for now.  Just treat it
  // as magic numbers, based on the recommendation from FIRST and WPILib.
  const RamseteConfig m_ramseteConfig;
};

inline frc2::SequentialCommandGroup*
TrajectoryCommandGenerator::GenerateCommandImpl(
    CommonDriveSubsystem* const drive, const DriveProfileData profileData,
    const PIDConfig pidConfig, const SpeedProfile speedProfile,
    const RamseteConfig ramseteConfig, const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {
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
              drive->ResetOdometry(trajectory.InitialPose());
            }
          },
          {drive}),
      std::move(ramseteCommand),
      frc2::InstantCommand([drive] { drive->TankDriveVolts(0_V, 0_V); },
                           {drive}));
}
