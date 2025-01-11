// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class SetRobotOdometry extends Command {
  Drivebase m_drivebase;
  Pose2d m_pose;
  /** Creates a new SetRobotOdometry. */
  public SetRobotOdometry(Drivebase drivebase, Pose2d pose) {
    m_drivebase = drivebase;
    m_pose = pose;
    System.out.println(pose.toString());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.resetOdometry(m_pose);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public boolean isFinished() {
    return true;
  }
}
