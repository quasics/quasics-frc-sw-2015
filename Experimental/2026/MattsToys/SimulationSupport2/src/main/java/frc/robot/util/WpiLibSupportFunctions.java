package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;

public class WpiLibSupportFunctions {
  /**
   * Helper function to allocate/configure a WPILib Encoder.
   *
   * @param portId1                   "A" port for the encoder
   * @param portId2                   "B" port for the encoder
   * @param inverted                  indicates if the encoder is inverted
   * @param outerDiameter             diameter of the thing being measured by
   *     the
   *                                  encoder (used to calculate distance per
   *                                  pulse)
   * @param encoderTicksPerRevolution number of encoder ticks per revolution of
   *                                  the wheel (used to calculate distance per
   *                                  pulse)
   * @return a configured encoder
   */
  public static Encoder getConfiguredEncoder(int portId1, int portId2,
      boolean inverted, Distance outerDiameter,
      double encoderTicksPerRevolution) {
    final Encoder encoder = new Encoder(portId1, portId2);
    encoder.setReverseDirection(inverted);
    configureEncoderForDistance(
        encoder, outerDiameter, encoderTicksPerRevolution);
    return encoder;
  }

  /**
   * Updates a SparkMaxConfig to work with distance-based values (meters and
   * meters/sec), rather than the native rotation-based units (rotations and
   * RPM).
   *
   * @param encoder                   the encoder being configured
   * @param outerDiameter             distance of the object (wheel, sprocket,
   *                                  etc.) being
   *                                  turned
   * @param encoderTicksPerRevolution number of encoder ticks per revolution of
   *                                  the wheel (used to calculate distance per
   *                                  pulse)
   */
  public static void configureEncoderForDistance(Encoder encoder,
      Distance outerDiameter, double encoderTicksPerRevolution) {
    encoder.setDistancePerPulse(
        Math.PI * outerDiameter.in(Meters) / encoderTicksPerRevolution);
  }
}
