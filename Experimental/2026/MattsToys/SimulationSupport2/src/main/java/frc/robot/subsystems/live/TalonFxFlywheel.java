// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.BaseFlywheel;
import frc.robot.util.config.FlywheelConfig;

/**
 * A simple flywheel subsystem that uses a TalonFX and WPILib's
 * SimpleMotorFeedforward to control the flywheel's velocity.
 *
 * This is a "live" subsystem, meaning it's meant for testing and
 * experimentation. It may not be fully optimized or robust, but it should
 * provide a good starting point for understanding how to use feedforward
 * control with a TalonFX.
 * 
 * For information on how to use SysId to determine the feedforward constants,
 * see the CTRE Phoenix 6 documentation and the WPILib documentation on
 * feedforward control.
 * 
 * Note: the base class defines speeds and feedforward in terms of RPM, but the
 * TalonFX expects velocity in RPS. This class handles the conversion
 * internally, so you can work with RPM values when using the setRPM and
 * getCurrentRPM methods.
 * 
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/sysid-integration/index.html
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
 */
public class TalonFxFlywheel extends BaseFlywheel {
  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  /**
   * Creates a new TalonFxFlywheel subsystem.
   * 
   * @param flywheelConfig the flywheel configuration to use.
   */
  public TalonFxFlywheel(FlywheelConfig flywheelConfig) {
    super(flywheelConfig.feedForward());

    motor = new TalonFX(flywheelConfig.motorID());

    // Note: I am leaving kFF at 0 because I am calculating it manually.
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = flywheelConfig.pidConfig().kP();
    configs.Slot0.kI = flywheelConfig.pidConfig().kI();
    configs.Slot0.kD = flywheelConfig.pidConfig().kD();

    motor.getConfigurator().apply(configs);
  }

  @Override
  public void setRPM(double targetRPM) {
    // Calculate the necessary voltage using WPILib's helper
    final double ffVoltage = feedforward.calculate(targetRPM);

    // Set the velocity request with the calculated feedforward voltage
    velocityRequest
        .withVelocity(targetRPM * 60) // Convert RPM to RPS (TalonFX expects RPS)
        .withFeedForward(ffVoltage);

    // Send the request to the TalonFX motor
    motor.setControl(velocityRequest);
  }

  @Override
  public double getCurrentRPM() {
    return motor.getVelocity().getValueAsDouble() * 60; // Convert RPS back to RPM
  }

  @Override
  public double getSetpointRPM() {
    return velocityRequest.Velocity * 60; // Convert RPS back to RPM
  }
}
