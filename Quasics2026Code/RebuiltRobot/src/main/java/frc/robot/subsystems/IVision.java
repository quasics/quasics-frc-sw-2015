package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

public interface IVision {
  record TargetData(int id, double yaw, double pitch, double distanceToTarget) {

  }

  boolean canSeeTargets();

  void setReferencePositionSupplier(Supplier<Pose2d> supplier);
}
