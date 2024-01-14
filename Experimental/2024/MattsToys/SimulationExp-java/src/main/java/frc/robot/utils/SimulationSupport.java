package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * During simulation, some data may need to be shared between subsystems that
 * wouldn't be used under "real" conditions.  This class is meant to provide a
 * simple clearing house for that data.  (There's better ways to handle this,
 * but this has the advantage of being simple....)
 */
public class SimulationSupport {
  static private Optional<Pose2d> simulatedPose = Optional.empty();

  public static synchronized Optional<Pose2d> getSimulatedPose() {
    return simulatedPose;
  }

  public static synchronized void setSimulatedPose(Pose2d pose) {
    if (pose != null) {
      simulatedPose = Optional.of(pose);
    } else {
      simulatedPose = Optional.empty();
    }
  }
}
