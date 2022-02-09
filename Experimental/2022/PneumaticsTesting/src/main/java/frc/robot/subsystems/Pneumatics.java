// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(
      // Hub type: must either be Rev or CTRE
      (Constants.kUsingRevRoboticsPneumaticHub ? PneumaticsModuleType.REVPH : PneumaticsModuleType.CTREPCM),
      // Forward and reverse channels
      Constants.kDoubleSolenoidForwardChannel, Constants.kDoubleSolenoidReverseChannel);

  /** Creates a the subsystem. */
  public Pneumatics() {
  }

  /**
   * Triggers extension of the double solenoid.
   */
  public void extend() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Triggers retraction of the double solenoid.
   */
  public void retract() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
