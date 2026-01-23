package frc.robot.subsystems;

public interface IVision {
  record TargetData(int id, double yaw, double pitch, double skew) {

  }

  boolean canSeeTargets();
}
