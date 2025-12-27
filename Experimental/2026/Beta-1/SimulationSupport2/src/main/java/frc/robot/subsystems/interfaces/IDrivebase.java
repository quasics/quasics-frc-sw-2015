package frc.robot.subsystems.interfaces;

/** Interface for a drivebase subsystem. */
public interface IDrivebase extends ISubsystem {
  String SUBSYSTEM_NAME = "Drivebase";

  /**
   * "Classic" arcade-style driving, based on percentages. (Note: operates
   * directly; no PID.)
   *
   * @param forward  forward speed (-1.0 to 1.0)
   * @param rotation rotation rate (-1.0 to 1.0)
   */
  void driveArcade(double forward, double rotation);

  /**
   * "Classic" tank-style driving, based on percentages. (Note: operates directly;
   * no PID.)
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
