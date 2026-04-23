// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.actuators;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;

/**
 * IMotorControllerPlus specialization for TalonFX motors.
 * 
 * Note: this code is (as yet) untested, as I don't have any TalonFX motors to
 * test it with. It may need some adjustments to work properly.
 * 
 * TODO: Test this code with a real TalonFX motor and make any necessary fixes.
 */
public class TalonMotorControllerPlus implements IMotorControllerPlus {
  /** Wrapped TalonFX motor. */
  final TalonFX m_talon;

  /** Constructor. */
  public TalonMotorControllerPlus(TalonFX talon) {
    m_talon = talon;
  }

  @Override
  public void disable() {
    m_talon.disable();
  }

  @Override
  public void set(double speed) {
    m_talon.set(speed);
  }

  @Override
  public double get() {
    return m_talon.get();
  }

  @Override
  public void setInverted(boolean isInverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = isInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    m_talon.getConfigurator().apply(config);
  }

  @Override
  public boolean getInverted() {
    TalonFXConfiguration currentConfigs = new TalonFXConfiguration();
    m_talon.getConfigurator().refresh(currentConfigs);
    return currentConfigs.MotorOutput.Inverted == InvertedValue.Clockwise_Positive;
  }

  @Override
  public void stopMotor() {
    m_talon.stopMotor();
  }

  //
  // IMotorControllerPlus methods
  //

  @Override
  public Voltage getVoltage() {
    return Volts.of(m_talon.getSupplyVoltage().getValueAsDouble());
  }

  @Override
  public void close() throws IOException {
    m_talon.close();
  }

  @Override
  public boolean canSetBrakeMode() {
    return true;
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = enabled
        ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;

    m_talon.getConfigurator().apply(config);
  }
}
