// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeployment extends SubsystemBase {

  /** Motor used to deploy/retract the floor intake. */
  final VictorSPX m_winchMotor = new VictorSPX(Constants.VictorCanIds.WINCH_MOTOR);

  /** Limit switch used to check if the floor intake is fully deployed. */
  final DigitalInput m_limitSwitch = new DigitalInput(Constants.DigitalIOIds.INTAKE_LIMIT_SWITCH);

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
