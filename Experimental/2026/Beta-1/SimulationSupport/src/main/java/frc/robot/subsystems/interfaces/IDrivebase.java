package frc.robot.subsystems.interfaces;

/** Interface for a drivebase subsystem. */
public interface IDrivebase extends ISubsystem {
  String SUBSYSTEM_NAME = "Drivebase";

  /**
   * Drives the robot using arcade controls.
   * 
   * @param forward  forward speed (-1.0 to 1.0)
   * @param rotation rotation rate (-1.0 to 1.0)
   */
  void driveArcade(double forward, double rotation);

  /**
   * Drives the robot using tank controls.
   * 
   * @param leftSpeed  speed for the left side (-1.0 to 1.0)
   * @param rightSpeed speed for the right side (-1.0 to 1.0)
   */
  void driveTank(double leftSpeed, double rightSpeed);

  /** Stops all motion of the drivebase. */
  default void stop() {
    driveTank(0.0, 0.0);
  }
}
