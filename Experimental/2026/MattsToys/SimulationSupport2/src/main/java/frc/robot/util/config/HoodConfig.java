package frc.robot.util.config;

import edu.wpi.first.units.measure.Angle;

public record HoodConfig(ControlType type, int canId, Angle minAngle, Angle maxAngle) {
  public enum ControlType {
    Null,
    SparkMax,
  }
}
