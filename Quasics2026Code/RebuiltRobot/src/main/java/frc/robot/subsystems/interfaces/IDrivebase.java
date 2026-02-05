// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public interface IDrivebase {
  public void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  public void setSpeeds(double leftSpeed, double rightSpeed);

  public double mpsToPercent(double speed);

  public Pose2d getOdometryPose();

  public Pose2d getEstimatedPose();

  public void resetOdometry(Pose2d pose);
}
