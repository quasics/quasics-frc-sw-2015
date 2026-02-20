// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IFlywheel;
import frc.robot.util.config.SimpleFeedForwardConfig;

/**
 * A base class for flywheel subsystems. This class provides common
 * functionality for flywheel subsystems, such as feedforward calculation (based
 * on RPM values) and debugging output.
 * 
 * This class is meant to be extended by specific flywheel implementations
 * (e.g., SparkMaxFlywheel, TalonFxFlywheel) that will provide the
 * hardware-specific details of how to set the RPM and read the current RPM.
 */
public abstract class BaseFlywheel extends SubsystemBase implements IFlywheel {

  /** If true, log debugging information to SmartDashboard. */
  private boolean m_debug = true;

  /** Feedforward controller for calculating voltage based on RPM. */
  protected final SimpleMotorFeedforward feedforward;

  /**
   * Constructor for BaseFlywheel. Initializes the feedforward controller with the
   * given constants.
   * 
   * @param feedforwardConfig a SimpleFeedForwardConfig object that has been
   *                          initialized with the appropriate kS, kV, and kA
   *                          constants for the flywheel
   */
  protected BaseFlywheel(SimpleFeedForwardConfig feedforwardConfig) {
    this(new SimpleMotorFeedforward(
        feedforwardConfig.kS().in(Volts),
        feedforwardConfig.kV().in(Volts),
        feedforwardConfig.kA()));
  }

  /**
   * Constructor for BaseFlywheel. Initializes the feedforward controller with the
   * given constants.
   * 
   * @param feedforward a SimpleMotorFeedforward object that has been initialized
   *                    with the appropriate kS, kV, and kA constants for the
   *                    flywheel
   */
  protected BaseFlywheel(SimpleMotorFeedforward feedforward) {
    this.feedforward = feedforward;
  }

  /**
   * Constructor for BaseFlywheel. Initializes the feedforward controller with the
   * given constants.
   * 
   * @param kS The static gain (Volts). This is the voltage required to overcome
   *           static friction and start the flywheel moving. For many flywheels,
   *           this can often be ignored (i.e., set to 0) because once the
   *           flywheel is spinning, it doesn't require much voltage to keep it
   *           going.
   * @param kV The velocity gain (Volts per RPM). This is the voltage required to
   *           maintain a certain RPM. For example, if kV is 0.002 Volts per RPM,
   *           then to maintain 3000 RPM, you would need 0.002 * 3000 = 6 Volts.
   * @param kA The acceleration gain (Volts per RPM/s). This is the voltage
   *           required to achieve a certain acceleration. For example, if kA is
   *           0.0001 Volts per RPM/s, then to accelerate at 1000 RPM/s, you would
   *           need 0.0001 * 1000 = 0.1 Volts.
   */
  protected BaseFlywheel(double kS, double kV, double kA) {
    // Initialize WPILib Feedforward with SysId constants (Volts)
    this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  /**
   * Enables or disables debug output to SmartDashboard. When enabled, the
   * periodic() method will log the current RPM, target RPM, and feedforward
   * voltage to SmartDashboard for debugging purposes.
   * 
   * @param tf True to enable debug output, false to disable it
   */
  public void enableDebug(boolean tf) {
    m_debug = tf;
  }

  //
  // SubsystemBase methods
  //

  @Override
  public void periodic() {
    if (m_debug) {
      final double targetRPM = getCurrentRPM();
      final double currentRPM = getSetpointRPM();
      final double ffVoltage = feedforward.calculate(targetRPM);
      SmartDashboard.putNumber("Flywheel RPM", currentRPM);
      SmartDashboard.putNumber("Flywheel Target RPM", targetRPM);
      SmartDashboard.putNumber("Flywheel FF Voltage", ffVoltage);
    }
  }

  //
  // IFlywheel interface methods --- these are abstract because the specific way
  // to set the RPM and read the current RPM will depend on the hardware
  // implementation (e.g., SparkMax vs. TalonFX).
  //

  @Override
  public abstract void setRPM(double targetRPM);

  @Override
  public abstract double getCurrentRPM();

  @Override
  public abstract double getSetpointRPM();
}
