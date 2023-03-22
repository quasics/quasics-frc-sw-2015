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
#include <wpi/SmallString.h>
#include <wpi/fs.h>

#include <exception>
#include <iostream>
#include <vector>

#include "Constants.h"
#include "subsystems/Drivebase.h"

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
    MetersPerSecond maxVelocity = 4.5623 * (1_m / 1_s);
    MetersPerSecondSquared maxAcceleration = 1.608 * (1_m / (1_s * 1_s));
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
    units::unit_t<frc::RamseteController::b_unit> kRamseteB{2};
    // Tuning parameter (0 < zeta < 1) for which larger values provide
    // more damping in response.
    units::unit_t<frc::RamseteController::zeta_unit> kRamseteZeta{0.7};
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
  TrajectoryCommandGenerator(Drivebase* drive,
                             const DriveProfileData& profileData,
                             const PIDConfig& pidConfig)
      : m_drive(drive), m_profileData(profileData), m_pidConfig(pidConfig) {
  }

  // Defines an enum class that gives the telemetry handling two states.
  enum TelemetryHandling {
    ResetTelemetryAtStart,
    UseExistingTelemetry,
  };

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
   * @param telemetryHandling
   *     if ResetTelemetryAtStart, the command will (at its initiation) reset
   *     the drive telemetry, allowing it to start following the trajectory
   *     using its current position and orientation as the origin point (0,0).
   *     Otherwise, it will use the previously-established origin as a
   *     starting point (and first drive back to that).
   */
  frc2::SequentialCommandGroup* GenerateCommand(
      const SpeedProfile speedProfile, const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, TelemetryHandling telemetryHandling) {
    return GenerateCommandFromDiscreteSegments(
        m_drive, m_profileData, m_pidConfig, speedProfile, start,
        interiorWaypoints, end, telemetryHandling, m_ramseteConfig);
  }  // uses explicit data given to the command to build a trajectory

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
   * @param telemetryHandling
   *     if ResetTelemetryAtStart, the command will (at its initiation) reset
   *     the drive telemetry, allowing it to start following the trajectory
   *     using its current position and orientation as the origin point (0,0).
   *     Otherwise, it will use the previously-established origin as a
   *     starting point (and first drive back to that).
   *
   * @return a sequential command group that includes directions to drive the
   * specified trajectory, or nullptr on an error (e.g., file couldn't be
   * found/read, was corrupted, etc.).
   */
  frc2::SequentialCommandGroup* GenerateCommandFromPathWeaverFile(
      const std::string jsonFileName, TelemetryHandling telemetryHandling) {
    const std::string directory = frc::filesystem::GetDeployDirectory();
    const std::string filename =
        directory + "/" + "paths" + "/" +
        jsonFileName;  // creates an adjusted location of the file

    try {
      frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(
          filename);  // converts the file given from pathweaver to something
                      // the code can use

      return GenerateCommandForTrajectory(
          m_drive, m_profileData, m_pidConfig, trajectory, telemetryHandling,
          m_ramseteConfig);  // calls the function created further in the code
    } catch (const std::exception& e) {
      // OK.  Something failed during this process (e.g., the specified JSON
      // file couldn't be found/read, etc.).  So we'll log the error to stderr,
      // and then return a nullptr to let the caller know.
      std::cerr
          << "**********************************************************\n"
          << "Failure in GenerateCommandFromPathWeaverFile(): " << e.what()
          << '\n'
          << "when loading file: " << filename << '\n'
          << "**********************************************************\n";
      return nullptr;
    }
  }  // this command takes in a file and uses it to build a trajectory

 private:
  // Having a static version helps to prevent accidental captures
  // of "this" in lambdas: if the generator is a local variable,
  // that's going to make the code go "boom" when you try to use
  // the command later....
  static frc2::SequentialCommandGroup* GenerateCommandFromDiscreteSegments(
      Drivebase* const drive, const DriveProfileData profileData,
      const PIDConfig pidConfig, const SpeedProfile speedProfile,
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, TelemetryHandling telemetryHandling,
      const RamseteConfig ramseteConfig);  // this is a .h file so these
                                           // describe what these functions take

  static frc2::SequentialCommandGroup* GenerateCommandForTrajectory(
      Drivebase* const drive, const DriveProfileData profileData,
      const PIDConfig pidConfig, frc::Trajectory trajectory,
      TelemetryHandling telemetryHandling,
      const RamseteConfig ramseteConfig);  // this is a .h file so these
                                           // describe what these functions take

 private:
  Drivebase* m_drive;
  const DriveProfileData m_profileData;
  const PIDConfig m_pidConfig;

  // I'm not even going to expose this for overriding for now.  Just treat it
  // as magic numbers, based on the recommendation from FIRST and WPILib.
  const RamseteConfig m_ramseteConfig;
};

// defines the functions here instead of in a .cpp file
// this generates a command from explicit data given
inline frc2::SequentialCommandGroup*
TrajectoryCommandGenerator::GenerateCommandFromDiscreteSegments(
    Drivebase* const drive, const DriveProfileData profileData,
    const PIDConfig pidConfig, const SpeedProfile speedProfile,
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, TelemetryHandling telemetryHandling,
    const RamseteConfig ramseteConfig) {
  const frc::DifferentialDriveKinematics kDriveKinematics{
      RobotPhysics::TRACK_WIDTH_INCHES_GLADYS};

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      profileData.kS, profileData.kV, profileData.kA);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);

  frc::TrajectoryConfig config(
      speedProfile.maxVelocity,
      speedProfile.maxAcceleration);  // the two following lines are configuring
                                      // the settings for a command to run by
  config.SetKinematics(
      kDriveKinematics);  // this is telling the command the width of the track
                          // beteween the motors
  config.AddConstraint(
      voltageConstraints);  /////This is adding a voltage constraint of 10V

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end,
      config);  // uses parameters given from this command to generate a
                // trajectory

  return GenerateCommandForTrajectory(
      drive, profileData, pidConfig, trajectory, telemetryHandling,
      ramseteConfig);  // takes in the trajectory(created above) and some other
                       // parameters and calls the function
}

// defines the functions here instead of in a .cpp file
// all calls of functions eventually end up here where it actually builds the
// RameseteCommand
inline frc2::SequentialCommandGroup*
TrajectoryCommandGenerator::GenerateCommandForTrajectory(
    Drivebase* const drive, const DriveProfileData profileData,
    const PIDConfig pidConfig, frc::Trajectory trajectory,
    TelemetryHandling telemetryHandling, RamseteConfig ramseteConfig) {
  const frc::DifferentialDriveKinematics kDriveKinematics{
      RobotPhysics::TRACK_WIDTH_INCHES_GLADYS};
  frc::SimpleMotorFeedforward<units::meter> feedForward(
      profileData.kS, profileData.kV, profileData.kA);

  // this should be the equivalent of the Ramesete command created in WPILIB
  // the above code is providing different data that is required and combining
  // some of the data

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
          [drive, telemetryHandling, trajectory] {
            if (telemetryHandling == ResetTelemetryAtStart) {
              std::cout << "Resetting robot odometry" << std::endl;
              drive->ResetOdometry(trajectory.InitialPose());
            }
          },
          {drive}),
      std::move(ramseteCommand),
      frc2::InstantCommand([drive] { drive->TankDriveVolts(0_V, 0_V); },
                           {drive}));
}
