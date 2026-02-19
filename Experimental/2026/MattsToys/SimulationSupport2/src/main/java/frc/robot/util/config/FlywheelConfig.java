package frc.robot.util.config;

public record FlywheelConfig(FlywheelType type, int motorID, boolean inverted) {
  public enum FlywheelType {
    SparkMax,
    TalonFX,
    Null
  }
}
