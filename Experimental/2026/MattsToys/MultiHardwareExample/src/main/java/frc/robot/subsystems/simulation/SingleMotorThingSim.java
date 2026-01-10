package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleMotorThing;

/**
 * An example of how the interface to the subsystem could be directly implemented.
 */
public class SingleMotorThingSim extends SubsystemBase implements ISingleMotorThing {
  /** Encoder ticks per revolution. */
  public static final int ENCODER_TICKS_PER_REVOLUTION = -4096;

  /** Wheel diameter in inches. */
  public static final Distance WHEEL_DIAMETER = Inches.of(6);

  private final PWMSparkMax m_motor = new PWMSparkMax(1);
  private final Encoder m_encoder = new Encoder(0, 1);

  protected static void configureEncoderForDistance(Encoder encoder, Distance outerDiameter) {
    encoder.setDistancePerPulse(Math.PI * WHEEL_DIAMETER.in(Meters) / ENCODER_TICKS_PER_REVOLUTION);
  }

  public SingleMotorThingSim() {
  }

  //
  // Methods from ISingleMotorThing
  //

  @Override
  public void setSpeed(double percent) {
    m_motor.set(percent);
  }

  @Override
  public double getSpeed() {
    return m_motor.get();
  }

  @Override
  public Distance getPosition() {
    return Meters.of(m_encoder.getDistance());
  }

  //
  // Methods from SubsystemBase
  //

  @Override
  public void simulationPeriodic() {
    // Do simulation things here...
  }
}
