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
#include <iostream>

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

  config.SetReversed(false);

  // An example trajectory to follow.  All units in meters.
  
  /*auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Translation2d{0_m, 0_m}
        //frc::Translation2d{-1_m, 1_m}
      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{1_m, 0_m, 0_deg},
      // Pass the config
      config);*/

   fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
   deployDirectory = deployDirectory / "paths" / fileToLoad.c_str();
   //Comment this back in and comment out the thing above to get the file
   frc::Trajectory exampleTrajectory  = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

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
  driveBase->ResetOdometry(exampleTrajectory.InitialPose());

  return std::move(ramseteCommand)
      .BeforeStarting(
          frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}));
}




frc2::CommandPtr BuildTrajectoryUsingAprilTags(frc::Pose2d startingPose, frc::Pose2d endingPose, bool driveForward, Drivebase* driveBase){
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

  config.SetReversed(!driveForward);

  // An example trajectory to follow.  All units in meters.
  
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      startingPose,
      {},
      endingPose,
      config);


    for (const auto& state : exampleTrajectory.States()) {
    std::cout << "  "
       << "t: " << state.t.value() << ", vel: " << state.velocity.value()
       << ", a: " << state.acceleration.value() << ", pose: ("
       << state.pose.X().value() << "," << state.pose.Y().value() << ")"
       << "\n";
  }
  std::cout << "}\n";
//IMPORTANT CHANGE******** CHANGED driveBase->GetPose() to driveBase->GetEstimatedPose()
  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      exampleTrajectory, [driveBase] { return driveBase->GetEstimatedPose(); }, // Displaying Get Pose in Dashboard
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
  driveBase->ResetOdometry(exampleTrajectory.InitialPose()); 

  return std::move(ramseteCommand)
      .BeforeStarting(
          frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}));
}


/*void PerliminarySetup(){

};*/



//PLEASE REMMEMBER WHEN BUILDING MANUALLY THE ROBOT RESETS IT PLOTS THE COORDINATE PLANE ON TOP OF ITSELF SO WH
//WHATEVER DIRECTION IT IS FACING BECOMES THE X DIRECTION
frc2::CommandPtr ManuallyBuildTrajectoryUsingStandardOdometry(frc::Pose2d startingPose, std::vector<frc::Translation2d>& interiorWaypoints, frc::Pose2d endingPose, bool driveForward, Drivebase* driveBase)
{
    
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

  config.SetReversed(!driveForward);

  // An example trajectory to follow.  All units in meters.
  
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      startingPose,
      // Pass through these two interior waypoints, making an 's' curve path
      interiorWaypoints,
      // End 3 meters straight ahead of where we started, facing forward
      endingPose,
      // Pass the config
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
  driveBase->ResetOdometry(exampleTrajectory.InitialPose());

  return std::move(ramseteCommand)
      .BeforeStarting(
          frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}))
      // Because Mr. Healy is professionally paranoid....
      .AndThen(frc2::cmd::RunOnce([driveBase] { driveBase->TankDriveVolts(0_V, 0_V); }, {}));
}