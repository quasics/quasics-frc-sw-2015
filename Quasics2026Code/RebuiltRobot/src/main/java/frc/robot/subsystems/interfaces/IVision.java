package frc.robot.subsystems.interfaces;

public interface IVision {
  record TargetData(int id, double yaw, double pitch, double distanceToTarget) {

  }

  boolean canSeeTargets();
}
