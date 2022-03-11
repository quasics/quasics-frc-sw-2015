// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeDeployment extends SubsystemBase {

  final VictorSPX m_winchMotor = new VictorSPX(3);
  final DigitalInput m_limitSwitch = new DigitalInput(9);

  /** Creates a new IntakeDeployment. */
  public IntakeDeployment() {
  }

  public void stop() {
    setMotorSpeed(0);
  }

  public void setMotorSpeed(double speedPercent) {
    m_winchMotor.set(VictorSPXControlMode.PercentOutput, speedPercent);
  }

  public boolean intakeIsDown() {
    return !m_limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Limit switch", m_limitSwitch.get() ? "open" : "closed");
  }
}
