// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

// com.ctre.phoenix.sensors.WPI_Pigeon2

public class Navigation extends SubsystemBase {
  private WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(Constants.PIGEON2_CAN_ID);
  private Pigeon2_Faults m_pigeonFaults = new Pigeon2_Faults();

  /** Creates a new ExampleSubsystem. */
  public Navigation() {
    // Should be a no-op.
    m_pigeon.calibrate();
  }

  public Gyro getGyro() {
    return m_pigeon;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // Update current info on faults.
    m_pigeon.getFaults(m_pigeonFaults);

    // Update Smart Dashboard (debugging output)
    SmartDashboard.putNumber("Current angle", m_pigeon.getAngle());
    SmartDashboard.putNumber("Pitch", m_pigeon.getPitch());
    SmartDashboard.putNumber("Roll", m_pigeon.getRoll());
    SmartDashboard.putNumber("Yaw", m_pigeon.getYaw());
  }

  public boolean getFault() {
    return m_pigeonFaults.hasAnyFault();
  }

  public String getFaultMessage() {
    if (!m_pigeonFaults.hasAnyFault())
      return "No faults";
    String retval = "";
    retval += m_pigeonFaults.APIError ? "APIError, " : "";
    retval += m_pigeonFaults.AccelFault ? "AccelFault, " : "";
    retval += m_pigeonFaults.BootIntoMotion ? "BootIntoMotion, " : "";
    retval += m_pigeonFaults.GyroFault ? "GyroFault, " : "";
    retval += m_pigeonFaults.HardwareFault ? "HardwareFault, " : "";
    retval += m_pigeonFaults.MagnetometerFault ? "MagnetometerFault, " : "";
    retval += m_pigeonFaults.ResetDuringEn ? "ResetDuringEn, " : "";
    retval += m_pigeonFaults.SaturatedAccel ? "SaturatedAccel, " : "";
    retval += m_pigeonFaults.SaturatedMag ? "SaturatedMag, " : "";
    retval += m_pigeonFaults.SaturatedRotVelocity ? "SaturatedRotVelocity, " : "";
    return retval;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
