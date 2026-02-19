// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IFlywheel;

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
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/sysid-integration/index.html
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html
 * @see https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
 */
public class TalonFxFlywheel extends SubsystemBase implements IFlywheel {
  private final TalonFX motor;
  private final SimpleMotorFeedforward ffModel;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // TODO: These constants should be determined experimentally using SysId or
  // another approach.
  private static final double kS = 0.1; // Volts (can often be ignored for flywheel velocity control)
  private static final double kV = 0.002; // Volts per RPM
  private static final double kA = 0.0001; // Volts per RPM
  private static final double kP = 0.11; // Small proportional gain for on-board loop

  public TalonFxFlywheel(int deviceId) {
    ffModel = new SimpleMotorFeedforward(kS, kV, kA);
    motor = new TalonFX(deviceId);

    // Set ONLY kP for the onboard loop. (If we see a persistant small gap between
    // the target RPM and what we're actually getting out of the subsystem, then I
    // can try adding a *tiny* bit of kI.)
    //
    // Note: I am leaving kFF at 0 because I am calculating it manually.
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = kP;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.0;

    motor.getConfigurator().apply(configs);
  }

  @Override
  public void setRPM(double targetRPM) {
    // Calculate the necessary voltage using WPILib's helper
    final double ffVoltage = ffModel.calculate(targetRPM);

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
