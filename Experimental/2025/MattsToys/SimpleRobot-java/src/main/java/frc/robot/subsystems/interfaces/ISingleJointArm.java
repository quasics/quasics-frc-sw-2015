package frc.robot.subsystems.interfaces;

public interface ISingleJointArm extends ISubsystem {
  ////////////////////////////////////////////////////////////////////////////////////
  // Values defining the arm's characteristics/physics
  ////////////////////////////////////////////////////////////////////////////////////

  final double ARM_OUT_ANGLE_RADIANS = Math.toRadians(180);
  final double ARM_UP_ANGLE_RADIANS = Math.toRadians(90);
  final double GEARING = 5 * 5 * 3 * 4.44; // Arbitrary (but needs to be enough for
                                           // simulated physics to work)
  final double ARM_LENGTH_METERS = 1.0; // Arbitrary
  final double ARM_MASS_KG = 4.0; // Arbitrary

  public void setTargetPositionInRadians(double targetPosition);
}
