// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.function.Supplier;

public interface IDrivebase {
  void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);


  // TODO(ROBERT): This should take a linear valocity the ability to control speed directly or percentage speed
  // void setSpeeds(LinearVelocity leftSpeed, LinearVelocity rightSpeed);
  void setSpeeds(double leftSpeed, double rightSpeed);

  // TODO(ROBERT): Create the ability to control speed directly or percentage speed
  // void setPercent(double leftPercent, double rightPersent);

  // TODO(ROBERT): This should take a unit
  // double mpsToPercent(LinearVelocity speed);
  double mpsToPercent(double speed);

  Pose2d getOdometryPose();

  Pose2d getEstimatedPose();

  void resetOdometry(Pose2d pose);

  default void stop() {
    setSpeeds(0, 0);
  }

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);
}
