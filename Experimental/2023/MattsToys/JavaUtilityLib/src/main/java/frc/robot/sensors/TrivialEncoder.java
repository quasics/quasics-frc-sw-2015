package frc.robot.sensors;

/**
 * Wrapper for the trivial functionality for encoders.
 * 
 * This is required, as the RelativeEncoder class provided by the Spark Max
 * controllers isn't *actually* a version of the WPILib Encoder type.
 */
public interface TrivialEncoder {
  /** Returns the distance recorded by the encoder (in meters). */
  double getPosition();

  /** Returns the current speed reported by the encoder (in meters/sec). */
  double getVelocity();

  /** Resets the encoder's distance. */
  void reset();
}
