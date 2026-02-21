// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Supplier;

public interface IDrivebase {
  void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed);

  // TODO(ROBERT): This should take a linear valocity the ability to control
  // speed directly or percentage speed
  void setSpeeds(LinearVelocity leftSpeed, LinearVelocity rightSpeed);

  // Used to set voltage directly to the motors (for characterization,
  // trajectory following, etc.)
  void setVoltages(Voltage leftVoltage, Voltage rightVoltage);

  void setPercent(double leftPercent, double rightPersent);

  double mpsToPercent(LinearVelocity speed);

  Pose2d getOdometryPose();

  Pose2d getEstimatedPose();

  void resetOdometry(Pose2d pose);

  default void stop() {
    setSpeeds(MetersPerSecond.of(0), MetersPerSecond.of(0));
  }

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);
}
