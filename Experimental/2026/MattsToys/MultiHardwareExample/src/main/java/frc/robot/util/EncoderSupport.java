package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;

public final class EncoderSupport {
  public static void configureEncoderForDistance(
      Encoder encoder, Distance outerDiameter, double ticksPerRevolution) {
    encoder.setDistancePerPulse(Math.PI * outerDiameter.in(Meters) / ticksPerRevolution);
  }
}
