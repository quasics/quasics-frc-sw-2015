// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants.PwmMotorIds;

public class Intake extends SubsystemBase {
  // BAG motor + Victor SPX
  private PWMVictorSPX m_intakeMotor = new PWMVictorSPX(PwmMotorIds.SHOOTER_MOTOR_ID);

  /** Creates a new Intake. */
  public Intake() {
  }

  /** Stops the intake motor. */
  public void stopIntake() {
    setIntakeMotorPower(0);
  }

  public void setIntakeMotorPower(double percent) {
    // Restrict the value to [-1.0 .. +1.0].
    percent = Math.max(-1.0, Math.min(+1.0, percent));

    // Set the power.
    m_intakeMotor.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
