// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.abstracts.AbstractElevator;

/** Add your docs here. */
public class SimulationUxSupport {

  public static final SimulationUxSupport instance = new SimulationUxSupport();

  private SimulationUxSupport() {
    // Simulation rendering setup.
    Mechanism2d rootMech2d = new Mechanism2d(
        9,
        AbstractElevator.MAX_SAFE_HEIGHT.in(Meters) * 1.15 // Leave a little room at the top
    );
    m_elevatorMech2d = rootMech2d.getRoot("Root", 3, 0)
        .append(new MechanismLigament2d("LeftClimber", 0, 90));

    // Publish Mechanism2d to SmartDashboard.
    // To show the visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", rootMech2d);
  }

  public enum DeviceStatus {
    Manual, AtSetpoint, NotAtSetpoint
  }

  public void updateElevator(Distance height, DeviceStatus status) {
    m_elevatorMech2d.setLength(height.in(Meters));
    switch (status) {
      case Manual:
        m_elevatorMech2d.setColor(NO_SETPOINT);
        break;
      case AtSetpoint:
        m_elevatorMech2d.setColor(AT_SETPOINT);
        break;
      case NotAtSetpoint:
        m_elevatorMech2d.setColor(NOT_AT_SETPOINT);
        break;
    }
  }

  /**
   * Mechanism2d visualization of the hardware (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_elevatorMech2d;

  /** Color used to render the elevator when running under manual control. */
  private final static Color8Bit NO_SETPOINT = new Color8Bit(255, 165, 0);

  /**
   * Color used to render the elevator when we've reached the target position .
   */
  private final static Color8Bit AT_SETPOINT = new Color8Bit(0, 255, 0);

  /**
   * Color used to render the elevator when we're driving towards the target
   * position .
   */
  private final static Color8Bit NOT_AT_SETPOINT = new Color8Bit(255, 0, 0);

}
