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
      const frc::Pose2d& end, bool resetTelemetryAtStart) {
          return GenerateCommand(m_drive, start, interiorWaypoints, end, resetTelemetryAtStart);
      }

private:
  // Having a static version helps to prevent accidental captures
  // of "this" in lambdas: if the generator is a local variable,
  // that's going to make the code go "boom" when you try to use
  // the command later....
  static frc2::SequentialCommandGroup* GenerateCommand(
      CommonDriveSubsystem* const drive, const frc::Pose2d& start,
      const std::vector<frc::Translation2d>& interiorWaypoints,
      const frc::Pose2d& end, bool resetTelemetryAtStart);

 private:
  CommonDriveSubsystem* m_drive;
};

inline frc2::SequentialCommandGroup* TrajectoryCommandGenerator::GenerateCommand(
    CommonDriveSubsystem* const drive, 
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end, bool resetTelemetryAtStart) {

  // Note: this code assumes the availability of specific namespace and constants from "Constants.h".
  using namespace RobotData::DriveConstants;
  using namespace RobotData::PathFollowingLimits;

  const frc::DifferentialDriveKinematics kDriveKinematics{
      drive->GetTrackWidth()};

  frc::SimpleMotorFeedforward<units::meter> feedForward(
      ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  frc::DifferentialDriveVoltageConstraint voltageConstraints(
      feedForward, kDriveKinematics, 10_V);
  frc::TrajectoryConfig config(kMaxSpeed, kMaxAcceleration);

  config.SetKinematics(kDriveKinematics);

  config.AddConstraint(voltageConstraints);

  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      start, interiorWaypoints, end, config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [drive]() { return drive->GetPose(); },
      frc::RamseteController{kRamseteB, kRamseteZeta}, feedForward,
      kDriveKinematics, [drive]() { return drive->GetWheelSpeeds(); },
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      frc2::PIDController(kPDriveVel, kIDriveVel, kDDriveVel),
      [drive](auto left, auto right) {
        drive->TankDriveVolts(left, right);
      },
      {drive});

  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand(
          [drive, resetTelemetryAtStart, exampleTrajectory] {
            if (resetTelemetryAtStart) {
              drive->ResetOdometry(exampleTrajectory.InitialPose());
            }
          },
          {drive}),
      std::move(ramseteCommand),
      frc2::InstantCommand([drive] { drive->TankDriveVolts(0_V, 0_V); },
                           {drive}));
}
