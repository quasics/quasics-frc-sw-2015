package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import java.util.function.Supplier;

public class SimulatedGyro implements Gyro {
  /**
   * Convenient trivial implementation for when we have nothing to do for some piece of the
   * simulation.
   */
  public static final Runnable TRIVIAL_RUNNABLE = () -> {};

  private final Runnable closeFunction;
  private final Runnable calibrateFunction;
  private final Runnable resetFunction;
  private final Supplier<Double> getAngleFcn;
  private final Supplier<Double> getRateFcn;

  public SimulatedGyro(
      Runnable closeFunction,
      Runnable calibrateFunction,
      Runnable resetFunction,
      Supplier<Double> getAngleFcn,
      Supplier<Double> getRateFcn) {
    this.closeFunction = closeFunction;
    this.calibrateFunction = calibrateFunction;
    this.resetFunction = resetFunction;
    this.getAngleFcn = getAngleFcn;
    this.getRateFcn = getRateFcn;
  }

  @Override
  public void close() throws Exception {
    this.closeFunction.run();
  }

  @Override
  public void calibrate() {
    this.calibrateFunction.run();
  }

  @Override
  public void reset() {
    this.resetFunction.run();
  }

  @Override
  public double getAngle() {
    return this.getAngleFcn.get();
  }

  @Override
  public double getRate() {
    return this.getRateFcn.get();
  }
}
