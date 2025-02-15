package frc.robot.subsystems.interfaces;

public interface ISingleJointArm extends ISubsystem {
  ////////////////////////////////////////////////////////////////////////////////////
  // Values defining the arm's characteristics/physics
  ////////////////////////////////////////////////////////////////////////////////////

  final double MAX_ANGLE_RADIANS = Math.toRadians(80);
  final double MIN_ANGLE_RADIANS = Math.toRadians(190);
  final double GEARING = 5 * 5 * 3 * 4.44; // Arbitrary (but needs to be enough for
                                           // simulated physics to work)
  final double ARM_LENGTH_METERS = 1.0; // Arbitrary
  final double ARM_MASS_KG = 4.0; // Arbitrary
  final double STARTING_ANGLE_RADIANS =
      (MIN_ANGLE_RADIANS - MAX_ANGLE_RADIANS) / 2 + MAX_ANGLE_RADIANS;
  final boolean SIMULATE_GRAVITY = true;

  public void setTargetPositionInRadians(double targetPosition);
}
