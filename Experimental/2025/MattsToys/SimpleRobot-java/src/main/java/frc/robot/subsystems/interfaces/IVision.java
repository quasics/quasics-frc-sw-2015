package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

/**
 * Simple vision subsystem interface.
 *
 * Allows clients to determine what targets are seen and basic information, but
 * doesn't do position identification.
 */
public interface IVision extends ISubsystem {
  record TargetData(int id, Angle angle, Distance distance) {
  }
  boolean hasTargetsInView();
  List<TargetData> getTargets();
}
