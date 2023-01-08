package frc.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SimulatedGyro implements Gyro {
  public interface DoubleFunction {
    double get();
  }

  private final Runnable closeFunction;
  private final Runnable calibrateFunction;
  private final Runnable resetFunction;
  private final DoubleFunction getAngleFcn;
  private final DoubleFunction getRateFcn;

  public SimulatedGyro(
      Runnable closeFunction,
      Runnable calibrateFunction,
      Runnable resetFunction,
      DoubleFunction getAngleFcn,
      DoubleFunction getRateFcn) {
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
