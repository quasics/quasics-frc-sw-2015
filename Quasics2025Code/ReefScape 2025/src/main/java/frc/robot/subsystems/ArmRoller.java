// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRoller extends SubsystemBase {
  private TalonFX m_kraken;

  /** Creates a new ArmRoller. */
  public ArmRoller() {
    m_kraken = new TalonFX(0);
    m_kraken.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setSpeed(double speed) {
    m_kraken.set(speed);
  }

  public void stop() {
    m_kraken.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Kraken current", m_kraken.getSupplyCurrent().getValue().in(Amps));
    // This method will be called once per scheduler run
  }
}
