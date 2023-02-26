package frc.robot.sensors;

import edu.wpi.first.wpilibj.Encoder;

/**
 * Wrapper for the trivial functionality for encoders.
 *
 * <p>This is required, as the RelativeEncoder class provided by the Spark Max controllers isn't
 * *actually* a version of the WPILib Encoder type.
 */
public interface TrivialEncoder {
  /** Returns the distance recorded by the encoder (in meters). */
  double getPosition();

  /** Returns the current speed reported by the encoder (in meters/sec). */
  double getVelocity();

  /** Resets the encoder's distance. */
  void reset();

  /** Creates a TrivialEncoder wrapper around a stock WPILib Encoder object. */
  public static TrivialEncoder forWpiLibEncoder(final Encoder encoder) {
    if (encoder == null) {
      throw new IllegalArgumentException("Null encoder");
    }

    return new TrivialEncoder() {
      @Override
      public double getPosition() {
        return encoder.getDistance();
      }

      @Override
      public double getVelocity() {
        return encoder.getRate();
      }

      @Override
      public void reset() {
        encoder.reset();
      }
    };
  }
}
