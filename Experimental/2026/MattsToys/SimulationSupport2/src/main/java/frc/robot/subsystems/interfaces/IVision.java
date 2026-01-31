// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

/**
 * Interface for a basic vision subsystem.
 */
public interface IVision extends ISubsystem {
  /** The "canonical" name of the Vision subsystem */
  static String SUBSYSTEM_NAME = "Vision";

  /**
   * Holds data about a detected vision target.
   *
   * @param id    fiducial ID for the target
   * @param angle yaw to the angle (negative values means that it's to left of
   *              camera center)
   */
  record TargetData(int id, Angle angle, Distance distance) {
  }

  /////////////////////////////////////////////////////////////////////////////
  // Abstract methods
  //

  /**
   * @return true if any targets are currently visible to the vision system
   */
  boolean hasTargetsInView();

  /**
   * Returns a list of visible targets, along with their IDs and angles.
   *
   * @param robotPose current position of the robot on the field (e.g., from
   *                  odometery), which is
   *                  used to compute robot-relative positioning of the targets
   */
  List<TargetData> getVisibleTargets(Pose2d robotPose);

  public class NullVision implements IVision {

    @Override
    public void close() throws IOException {
      // No-op.
    }

    @Override
    public boolean hasTargetsInView() {
      return false;
    }

    @Override
    public List<TargetData> getVisibleTargets(Pose2d robotPose) {
      return Collections.emptyList();
    }

  }
}
