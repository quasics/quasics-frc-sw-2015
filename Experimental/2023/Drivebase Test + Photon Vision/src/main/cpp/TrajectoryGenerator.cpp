#include "TrajectoryGenerator.h"

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include "Constants.h"
#include "subsystems/Drivebase.h"

frc2::CommandPtr GetCommandForTrajectory(std::string fileToLoad, Drivebase* driveBase) {
// Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
      frc::SimpleMotorFeedforward<units::meters>{
          PathWeaverConstants::kS, PathWeaverConstants::kV, PathWeaverConstants::kA},
      kDriveKinematics, 10_V};

  // Set up config for trajectory
  frc::TrajectoryConfig config{kMaxSpeed, kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  config.SetReversed(true);

  // An example trajectory to follow.  All units in meters.
  
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d{1_m, 1_m}
        //frc::Translation2d{-1_m, 1_m}
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{-2_m, 0_m, 0_deg},
      // Pass the config
      config);

   fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
   deployDirectory = deployDirectory / "paths" / fileToLoad.c_str();
   // frc::Trajectory exampleTrajectory  = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      exampleTrajectory, [driveBase] { return driveBase->GetPose(); }, // Displaying Get Pose in Dashboard
      frc::RamseteController{kRamseteB, kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          PathWeaverConstants::kS, PathWeaverConstants::kV, PathWeaverConstants::kA},
      kDriveKinematics,
      [driveBase] { return driveBase->GetWheelSpeeds(); },
      frc2::PIDController{PathWeaverConstants::kP, 0, 0},
      frc2::PIDController{PathWeaverConstants::kP, 0, 0},
      [driveBase](auto left, auto right) { driveBase->TankDriveVolts(left, right); },
      {driveBase})}; 

  // Reset odometry to the starting pose of the trajectory.
  driveBase->ResetOdometry(exampleTrajectory.InitialPose()); // maybe here is the fundamental issue?????

  return std::move(ramseteCommand)
      .BeforeStarting(
          frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}));
}




frc2::CommandPtr BuildTrajectory(frc::Pose2d startingPose, frc::Pose2d endingPose, bool driveForward, Drivebase* driveBase){
    // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
      frc::SimpleMotorFeedforward<units::meters>{
          PathWeaverConstants::kS, PathWeaverConstants::kV, PathWeaverConstants::kA},
      kDriveKinematics, 10_V};

  // Set up config for trajectory
  frc::TrajectoryConfig config{kMaxSpeed, kMaxAcceleration};
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  config.SetReversed(driveForward);

  // An example trajectory to follow.  All units in meters.
  
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      startingPose,
      {},
      endingPose,
      config);

  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      exampleTrajectory, [driveBase] { return driveBase->GetPose(); }, // Displaying Get Pose in Dashboard
      frc::RamseteController{kRamseteB, kRamseteZeta},
      frc::SimpleMotorFeedforward<units::meters>{
          PathWeaverConstants::kS, PathWeaverConstants::kV, PathWeaverConstants::kA},
      kDriveKinematics,
      [driveBase] { return driveBase->GetWheelSpeeds(); },
      frc2::PIDController{PathWeaverConstants::kP, 0, 0},
      frc2::PIDController{PathWeaverConstants::kP, 0, 0},
      [driveBase](auto left, auto right) { driveBase->TankDriveVolts(left, right); },
      {driveBase})}; 

  // Reset odometry to the starting pose of the trajectory.
  driveBase->ResetOdometry(exampleTrajectory.InitialPose()); // maybe here is the fundamental issue?????

  return std::move(ramseteCommand)
      .BeforeStarting(
          frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}));
}


/*void PerliminarySetup(){

};*/