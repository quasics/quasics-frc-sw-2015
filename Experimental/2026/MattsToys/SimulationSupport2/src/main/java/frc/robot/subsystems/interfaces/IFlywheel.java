package frc.robot.subsystems.interfaces;

public interface IFlywheel {
  public void setRPM(double targetRPM);

  public double getCurrentRPM();

  public double getSetpointRPM();

  default void stop() {
    setRPM(0);
  }
}
