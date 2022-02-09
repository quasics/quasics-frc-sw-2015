// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntake extends SubsystemBase {
  private final VictorSPX m_motor = new VictorSPX(Constants.VictorCanIds.CONVEYOR_MOTOR);

  /** Creates a new FloorIntake. */
  public FloorIntake() {
    setName("Floor intake");
  }

  public void stop() {
    setSpeed(0.0);
  }

  public void setSpeed(double percentSpeed) {
    percentSpeed = Math.max(-1, Math.min(+1, percentSpeed));

    m_motor.set(ControlMode.PercentOutput, percentSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
