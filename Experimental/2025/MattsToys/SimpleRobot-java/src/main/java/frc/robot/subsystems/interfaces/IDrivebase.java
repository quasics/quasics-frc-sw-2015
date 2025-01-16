// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public interface IDrivebase {
  void setSpeed(double leftPercentage, double rightPercentage);

  // Utility method: straight forward/backward
  default void setSpeed(double percentage) {
    setSpeed(percentage, percentage);
  }

  // Utility method: stop
  default void stop() {
    setSpeed(0, 0);
  }

  Distance getLeftPositionMeters();

  Distance getRightPositionMeters();

  /** @return heading of the robot (as an Angle) */
  Angle getHeading();

  default Subsystem asSubsystem() {
    return (Subsystem) this;
  }
}
