package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose2d;

public interface IDrivebasePlus extends IDrivebase {
  public Pose2d getEstimatedPose();
}
