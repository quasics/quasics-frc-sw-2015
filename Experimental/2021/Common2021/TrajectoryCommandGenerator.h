#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/RamseteCommand.h>

// #include <frc2/command/CommandHelper.h>

#include <vector>

// In "Common2021"
#include "CommonDriveSubsystem.h"

// Project-specific
//
// TODO(mjh): Remove assumptions about this.
#include "Constants.h"

class TrajectoryCommandGenerator {
 public:
  TrajectoryCommandGenerator(CommonDriveSubsystem* drive) : m_drive(drive) {}
  frc2::SequentialCommandGroup* GenerateCommand(
      const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart);

 private:
  CommonDriveSubsystem* m_drive;
};

inline frc2::SequentialCommandGroup* TrajectoryCommandGenerator::GenerateCommand(
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {

  // Note: this code assumes the availability of specific namespace and constants from "Constants.h".
  using namespace RobotData::DriveConstants;
  using namespace RobotData::PathFollowingLimits;

  const frc::DifferentialDriveKinematics kDriveKinematics{
      m_drive->GetTrackWidth()};

  // Set up config for trajectory
  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);
  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);

  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);

  // Apply the voltage constraint.  (Don't draw more than is safe!)
  config.AddConstraint(voltageConstraints);

  // Generate the trajectory.
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  // Build the core command to follow the trajectory.
  frc2::RamseteCommand ramseteCommand(
      /* trajectory to follow */
      trajectory,
      /* function that supplies the robot pose */
      [this]() { return m_drive->GetPose(); },
      /* RAMSETE controller used to follow the trajectory */
      frc::RamseteController(kRamseteB, kRamseteZeta),
      /* calculates the feedforward for the drive */
      feedForward,
      /* kinematics for the robot drivetrain */
      kDriveKinematics,
      /* function that supplies the left/right side speeds */
      [this] { return m_drive->GetWheelSpeeds(); },
      /* left controller */
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      /* right controller */
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      /* output function */
      [this](auto left, auto right) { m_drive->TankDriveVolts(left, right); },
      /* required subsystems */
      {m_drive});

  // Finally, build the full command sequence.
  return new frc2::SequentialCommandGroup(
      frc2::PrintCommand("Starting trajectory code"),
      frc2::InstantCommand([this, resetTelemetryAtStart, trajectory] {
        if (resetTelemetryAtStart) {
          m_drive->ResetOdometry(trajectory.InitialPose());
        }
      }),
      std::move(ramseteCommand),
      // Shut the drive down.
      frc2::PrintCommand("Shutting down trajectory code"),
      frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {}));
}
