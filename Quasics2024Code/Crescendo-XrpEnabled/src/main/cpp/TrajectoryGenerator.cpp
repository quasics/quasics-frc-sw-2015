#include "TrajectoryGenerator.h"

#include <frc/Filesystem.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <wpi/fs.h>

#include "Constants.h"
#include "subsystems/IDrivebase.h"

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad,
                                         IDrivebase* driveBase,
                                         bool reversed = false) {
  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
      frc::SimpleMotorFeedforward<units::meters>{PathWeaverConstants::kS,
                                                 PathWeaverConstants::kV,
                                                 PathWeaverConstants::kA},
      kDriveKinematics, 10_V};

  // Set up config for trajectory
  frc::TrajectoryConfig config{kMaxSpeed, kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  config.SetReversed(reversed);

  // An example trajectory to follow.  All units in meters.
  /*
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d{7_m, 0_m, 0_deg},
        // Pass through these two interior waypoints, making an 's' curve path
        {
            // frc::Translation2d{1_m, 0_m},
            // frc::Translation2d{-1_m, 1_m}
        },
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d{10_m, 0_m, 0_deg},
        // Pass the config
        config);
  */

  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / fileToLoad.c_str();
  frc::Trajectory exampleTrajectory =
      frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  frc::Transform2d transform =
      driveBase->getOdometry().GetPose() - exampleTrajectory.InitialPose();
  frc::Trajectory newTrajectory = exampleTrajectory.TransformBy(transform);

  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      newTrajectory,
      [driveBase] {
        return driveBase->getPose();
      },  // Displaying Get Pose in Dashboard
      frc::RamseteController{kRamseteB, kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{PathWeaverConstants::kS,
                                                 PathWeaverConstants::kV,
                                                 PathWeaverConstants::kA},
      kDriveKinematics, [driveBase] { return driveBase->getWheelSpeeds(); },
      frc::PIDController{PathWeaverConstants::kP, 0, 0},
      frc::PIDController{PathWeaverConstants::kP, 0, 0},
      [driveBase](auto left, auto right) {
        driveBase->tankDriveVolts(left, right);
      },
      {driveBase})};

  return std::move(ramseteCommand)
      .BeforeStarting(frc2::cmd::RunOnce(
          [driveBase] { driveBase->tankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce(
          [driveBase] { driveBase->tankDriveVolts(0_V, 0_V); }, {}));
}
